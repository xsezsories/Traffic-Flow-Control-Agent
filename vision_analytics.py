"""
vision_analytics.py
====================
วัดสถานะการจราจรต่อเลน (vehicle_count, occupancy_ratio, avg_speed_kmh)
รอบสี่แยกที่กำหนด พร้อมตรวจจับรถฉุกเฉิน (TC-03) และรถติดค้างนิ่ง (TC-04)

หมายเหตุ: ตอนนี้อ่านตำแหน่ง/ความเร็วรถจาก CARLA actor API ตรงๆ (ground truth)
เป็น placeholder แทนโมเดล CV จริง — เปลี่ยนมาใช้ผลตรวจจับจากภาพ (YOLO ฯลฯ) ทีหลังได้
โดยไม่ต้องแก้ traffic_controller.py เพราะ output schema (lanes_status) เหมือนเดิม

จุดที่ต้องตรวจสอบเอง: การแมป CARLA world axis (X, Y) เป็นทิศ NORTH/SOUTH/EAST/WEST
ในฟังก์ชัน classify_direction() เป็นการเดาจากทิศแกนมาตรฐาน อาจไม่ตรงกับทิศจริงบนแมพ
ให้เช็คจาก world.debug.draw_string ที่วาดกำกับไว้ตอน build_lane_zones() ในหน้าต่าง
CARLA Spectator ว่า label ตรงกับทิศจริงไหม ถ้ากลับด้าน ให้สลับเครื่องหมายในฟังก์ชันนั้น
"""

import carla
import time

AVG_VEHICLE_LENGTH_M = 4.5      # ค่าประมาณความยาวรถเฉลี่ย ใช้คำนวณ occupancy_ratio
STATIONARY_SPEED_THRESHOLD = 0.3  # m/s ต่ำกว่านี้ถือว่า "หยุดนิ่ง"
INCIDENT_STOPPED_SEC = 20.0     # ตาม TC-04: หยุดนิ่ง >20s ในเลน active ถือเป็น incident
BOX_BUFFER_M = 8.0               # ระยะเผื่อครอบคลุมเข้าไปในกล่องกลางแยก (กันรถที่ขยับ
                                  # ล้ำเส้นหยุดไปแล้ว หรือรถ "block the box" หลุดจากการนับ)

EMERGENCY_VEHICLE_TYPES = {
    'vehicle.carlamotors.firetruck',
    'vehicle.ford.ambulance',
    'vehicle.dodge.charger_police',
    'vehicle.dodge.charger_police_2020',
}


def classify_direction(forward_vector):
    """ เดาทิศที่รถ 'มาจาก' (inbound) ตามมาตรฐานงานจราจร
        เช่น NORTH_INBOUND = รถที่มาจากทิศเหนือ วิ่งลงใต้เข้าแยก
        forward_vector คือทิศทางที่รถ 'กำลังวิ่งไป' (เข้าหาแยก) — ต้องกลับเครื่องหมาย
        ก่อนแปลงเป็นทิศ เพราะทิศที่รถมาจาก คือทิศตรงข้ามกับทิศที่รถวิ่งไป """
    from_direction = carla.Vector3D(-forward_vector.x, -forward_vector.y, -forward_vector.z)
    if abs(from_direction.x) > abs(from_direction.y):
        return 'EAST_INBOUND' if from_direction.x > 0 else 'WEST_INBOUND'
    else:
        return 'NORTH_INBOUND' if from_direction.y < 0 else 'SOUTH_INBOUND'


def build_lane_zones(world, junction, zone_length=30.0, debug_draw=True):
    """ สร้างโซนตรวจจับต่อเลนขาเข้าสี่แยก จาก topology จริงของ junction """
    lane_pairs = junction.get_waypoints(carla.LaneType.Driving)
    zones = {}

    print(f"[VISION] junction.get_waypoints() คืนมา {len(lane_pairs)} คู่ waypoint (ก่อน dedup)")

    for wp_enter, _wp_exit in lane_pairs:
        prev = wp_enter.previous(0.1)
        # บาง lane อาจอยู่ติดจุดเริ่มถนนพอดี ทำให้ previous() คืนค่าว่าง
        # แทนที่จะข้ามไปเฉยๆ (ซึ่งจะทำให้เลนนั้นหายไปทั้งทิศ) ใช้ wp_enter ตรงๆ แทน
        entry_point = prev[0] if prev else wp_enter
        fwd = entry_point.transform.get_forward_vector()
        direction = classify_direction(fwd)
        lane_key = f"{direction}_R{entry_point.road_id}_L{entry_point.lane_id}"

        if lane_key in zones:
            continue  # กันซ้ำ (หลาย lane pair อาจชี้มาจุดเดียวกัน)

        zones[lane_key] = {
            'entry_location': entry_point.transform.location,
            'entry_waypoint': entry_point,
            'forward': fwd,
            'lane_width': entry_point.lane_width,
            'zone_length': zone_length,
        }

        if debug_draw:
            end_point = entry_point.transform.location - fwd * zone_length
            world.debug.draw_arrow(
                entry_point.transform.location + carla.Location(z=1.0),
                end_point + carla.Location(z=1.0),
                thickness=0.15, arrow_size=0.3,
                color=carla.Color(0, 255, 255), life_time=0.0
            )
            world.debug.draw_string(
                end_point + carla.Location(z=2.0),
                lane_key, color=carla.Color(255, 255, 0), life_time=0.0
            )

    return zones


class LaneMonitor:
    """ เรียก update() ทุก tick เพื่อดึงสถานะจราจรปัจจุบันของทุกเลน """

    def __init__(self, world, junction, intersection_id="INT-CARLA-TOWN01", zone_length=30.0):
        self.world = world
        self.zones = build_lane_zones(world, junction, zone_length=zone_length)
        self.intersection_id = intersection_id
        self._stopped_since = {}  # actor_id -> timestamp ที่เริ่มหยุดนิ่ง (สำหรับ TC-04)

    def _vehicles_in_zone(self, zone):
        entry_loc = zone['entry_location']
        fwd = zone['forward']
        half_width = zone['lane_width'] / 2.0 + 0.5
        length = zone['zone_length']
        found = []

        for v in self.world.get_actors().filter('vehicle.*'):
            loc = v.get_location()
            rel_x = loc.x - entry_loc.x
            rel_y = loc.y - entry_loc.y
            along = -(rel_x * fwd.x + rel_y * fwd.y)   # ระยะถอยหลังจากจุดเข้าแยก ตามแนวเลน
            lateral = abs(rel_x * (-fwd.y) + rel_y * fwd.x)  # ระยะเบี่ยงด้านข้างจากกึ่งกลางเลน
            # ครอบตั้งแต่ -BOX_BUFFER_M (ล้ำเข้าไปในกล่องกลางแยกได้เล็กน้อย) ถึง zone_length
            if -BOX_BUFFER_M <= along <= length and lateral <= half_width:
                found.append(v)
        return found

    def update(self, active_green_group=None):
        """ คืน (lanes_status_list, emergency_info, incident_info)
            active_green_group: "NS" หรือ "EW" ที่ไฟกำลังเขียวอยู่ตอนนี้ (จาก TrafficController)
            ใช้เช็คว่ารถที่หยุดนิ่ง >20s อยู่ในเลนที่ 'ควรจะขยับได้แล้ว' (ไฟเขียว) จริงไหม
            ถ้าไม่ส่งมาหรือเป็น None (เช่นตอนไฟเหลือง/all-red) จะไม่นับเป็น incident เลย
            เพราะรถที่จอดรอไฟแดงตามปกติไม่ใช่อุบัติเหตุ ถึงจะรอนานเกิน 20 วิก็ตาม """
        now = time.time()
        lanes_status = []
        emergency_info = {"emergency_detected": False, "lane_id": None}
        incident_info = {"status": False, "lane_id": None, "stopped_duration_sec": 0}

        active_vehicle_ids_this_tick = set()

        for lane_id, zone in self.zones.items():
            vehicles = self._vehicles_in_zone(zone)
            speeds_kmh = []

            lane_is_active_green = (
                active_green_group == "NS" and lane_id.startswith(("NORTH_INBOUND", "SOUTH_INBOUND"))
            ) or (
                active_green_group == "EW" and lane_id.startswith(("EAST_INBOUND", "WEST_INBOUND"))
            )

            for v in vehicles:
                active_vehicle_ids_this_tick.add(v.id)
                vel = v.get_velocity()
                speed_ms = (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5
                speed_kmh = speed_ms * 3.6
                speeds_kmh.append(speed_kmh)

                # ---- TC-03: emergency vehicle preemption ----
                if v.type_id in EMERGENCY_VEHICLE_TYPES:
                    emergency_info = {"emergency_detected": True, "lane_id": lane_id}

                # ---- TC-04: incident / obstruction detection ----
                # นับเป็น incident เฉพาะรถที่หยุดนิ่ง >20s "ในเลนที่ไฟเขียวอยู่" เท่านั้น
                # (รถจอดรอไฟแดงตามปกติไม่ใช่อุบัติเหตุ ไม่ว่าจะรอนานแค่ไหน)
                if speed_ms < STATIONARY_SPEED_THRESHOLD:
                    started = self._stopped_since.setdefault(v.id, now)
                    stopped_duration = now - started
                    if lane_is_active_green and stopped_duration > INCIDENT_STOPPED_SEC:
                        incident_info = {
                            "status": True,
                            "lane_id": lane_id,
                            "stopped_duration_sec": round(stopped_duration, 1)
                        }
                else:
                    self._stopped_since.pop(v.id, None)

            vehicle_count = len(vehicles)
            effective_length = zone['zone_length'] + BOX_BUFFER_M
            occupancy_ratio = min(1.0, (vehicle_count * AVG_VEHICLE_LENGTH_M) / effective_length)
            avg_speed = round(sum(speeds_kmh) / len(speeds_kmh), 1) if speeds_kmh else 0.0

            lanes_status.append({
                "lane_id": lane_id,
                "vehicle_count": vehicle_count,
                "occupancy_ratio": round(occupancy_ratio, 2),
                "avg_speed_kmh": avg_speed,
            })

        # เคลียร์ timer ของรถที่ออกนอกทุกโซนไปแล้ว กันข้อมูลค้าง
        for vid in list(self._stopped_since.keys()):
            if vid not in active_vehicle_ids_this_tick:
                self._stopped_since.pop(vid, None)

        return lanes_status, emergency_info, incident_info

    def build_mqtt_payload(self, lanes_status, emergency_info, incident_info,
                            weather_id="CLEAR_NOON", visibility_score=0.98):
        """ ประกอบ payload ตาม schema ที่กำหนดไว้ใน spec จากผลที่ update() คำนวณไว้แล้ว
            (ไม่เรียก update() ซ้ำเพื่อไม่ให้เสียเวลาคำนวณซ้ำโดยไม่จำเป็นต่อ tick)
            ยังไม่ส่ง MQTT จริง (ส่วนต่อ AWS IoT Core / ESP32 ทำทีหลังวันที่ 18-19 ก.ย.) """
        payload = {
            "timestamp": int(time.time()),
            "intersection_id": self.intersection_id,
            "environment": {
                "weather": weather_id,
                "visibility_score": visibility_score,
            },
            "lanes_status": lanes_status,
            "emergency_detected": emergency_info["emergency_detected"],
            "incident_detected": incident_info,
        }
        return payload