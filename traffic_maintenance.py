"""
traffic_maintenance.py
========================
1. เติมรถให้คงจำนวนเป้าหมายไว้ตลอด (ไม่ใช่ spawn ทีเดียวตอนเริ่มแล้วจบ)
   เพราะรถอาจขับออกนอกแมพ/หายไปเรื่อยๆ ตามธรรมชาติของ CARLA traffic manager

2. ตรวจจับรถที่ "ค้างผิดปกติ" กลางถนน (ไม่ขยับเลยนานเกินกำหนด) แล้วทำลายทิ้ง
   พร้อม spawn คันใหม่ทดแทนที่อื่น เพื่อไม่ให้เกิดการจราจรค้างตายถาวร
   *ไม่นับรถที่จอดรอไฟแดงตามปกติใกล้ junction ที่เรามอนิเตอร์อยู่ว่าเป็นรถ "ค้าง"*
"""

import random
import time

# คงไว้ให้ตรงกับ EXCLUDED_LARGE_VEHICLES ใน carla_camera_node.py
EXCLUDED_VEHICLE_TYPES = [
    'vehicle.carlamotors.firetruck',
    'vehicle.carlamotors.carlacola',
    'vehicle.carlamotors.european_hgv',
    'vehicle.mitsubishi.fusorosa',
    'vehicle.mercedes.sprinter',
    'vehicle.ford.ambulance',
    'vehicle.volkswagen.t2',
    'vehicle.volkswagen.t2_2021',
    'vehicle.dodge.charger_police',
    'vehicle.dodge.charger_police_2020',
]

STUCK_TIME_SEC = 45.0       # ไม่ขยับเกินนี้ (นอกรัศมีป้องกันรอบ junction) ถือว่า "ค้างผิดปกติ"
STUCK_DISTANCE_M = 2.0      # ขยับน้อยกว่านี้ในช่วง STUCK_TIME_SEC ถือว่านิ่ง
REPLENISH_INTERVAL_SEC = 10.0


class TrafficMaintainer:
    def __init__(self, world, client, target_vehicle_count,
                 junction_center=None, protected_radius=45.0):
        self.world = world
        self.client = client
        self.traffic_manager = client.get_trafficmanager()
        self.target_count = target_vehicle_count
        self.junction_center = junction_center     # carla.Location ของ junction ที่มอนิเตอร์
        self.protected_radius = protected_radius   # รัศมีรอบ junction ที่ถือว่า "รอไฟปกติ" ไม่เช็คค้าง
        self._last_check = {}   # actor_id -> (timestamp, location)
        self._last_replenish = time.time()

    def _vehicle_blueprints(self):
        return [bp for bp in self.world.get_blueprint_library().filter('vehicle.*')
                if bp.id not in EXCLUDED_VEHICLE_TYPES]

    def _spawn_n(self, n):
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)
        bps = self._vehicle_blueprints()
        spawned = 0
        for point in spawn_points:
            if spawned >= n:
                break
            bp = random.choice(bps)
            v = self.world.try_spawn_actor(bp, point)
            if v is not None:
                v.set_autopilot(True, self.traffic_manager.get_port())
                spawned += 1
        return spawned

    def update(self):
        now = time.time()

        # ---- 1) เติมรถให้ครบเป้าเรื่อยๆ ----
        if now - self._last_replenish >= REPLENISH_INTERVAL_SEC:
            current_count = len(self.world.get_actors().filter('vehicle.*'))
            deficit = self.target_count - current_count
            if deficit > 0:
                added = self._spawn_n(deficit)
                if added > 0:
                    print(f"[MAINTAIN] เติมรถเพิ่ม {added} คัน (รวมตอนนี้ ~{current_count + added} คัน)")
            self._last_replenish = now

        # ---- 2) ตรวจจับรถที่ค้าง/ชะงักผิดปกติ แล้วกู้คืน ----
        vehicles = self.world.get_actors().filter('vehicle.*')
        active_ids = set()

        for v in vehicles:
            active_ids.add(v.id)
            loc = v.get_location()

            # ใกล้ junction ที่มอนิเตอร์ = รอไฟแดงได้ตามปกติ ไม่นับว่าค้าง
            if self.junction_center is not None:
                dx = loc.x - self.junction_center.x
                dy = loc.y - self.junction_center.y
                if (dx * dx + dy * dy) ** 0.5 <= self.protected_radius:
                    self._last_check[v.id] = (now, loc)
                    continue

            if v.id not in self._last_check:
                self._last_check[v.id] = (now, loc)
                continue

            last_time, last_loc = self._last_check[v.id]
            moved = ((loc.x - last_loc.x) ** 2 + (loc.y - last_loc.y) ** 2) ** 0.5

            if moved > STUCK_DISTANCE_M:
                self._last_check[v.id] = (now, loc)  # ขยับแล้ว รีเซ็ต baseline
            elif now - last_time > STUCK_TIME_SEC:
                print(f"[MAINTAIN] พบรถค้างผิดปกติ (id={v.id}) นอกรัศมี junction -> ทำลายแล้ว respawn ทดแทน")
                try:
                    v.destroy()
                except RuntimeError:
                    pass
                self._last_check.pop(v.id, None)
                self._spawn_n(1)

        for vid in list(self._last_check.keys()):
            if vid not in active_ids:
                self._last_check.pop(vid, None)