"""
traffic_controller.py
======================
State machine ควบคุมสัญญาณไฟจราจรของสี่แยก ตาม test case ที่กำหนดไว้:

  TC-01 Off-Peak            -> วนไฟเร็ว ไม่ค้างไฟเขียวถ้าไม่มีรถ (minimize idle delay)
  TC-02 Peak Asymmetric     -> จัดสรรเวลาไฟเขียวตามความหนาแน่นของแต่ละฝั่ง
                               แต่มี MIN_GREEN กันไม่ให้ฝั่งรถน้อยถูกทิ้ง (starve)
  TC-03 Emergency Preemption-> ถ้าเจอรถฉุกเฉิน ตัดไปเขียวให้ฝั่งนั้นทันที
  TC-04 Incident/Obstruction-> ถ้ามีรถติดค้าง >20s ในเลน active แจ้ง anomaly
                               และลดสัดส่วนเวลาไฟที่จัดให้ฝั่งที่ติดขัด

หมายเหตุ: ต้องรัน CARLA จริงเพื่อทดสอบพฤติกรรมไฟจริง ผมเขียน logic ตามสเปกให้ครบ
แต่ยังไม่เคยรันจริงบน simulator ของคุณ ควรทดสอบแล้วปรับค่าคงที่ (MIN_GREEN, MAX_GREEN
ฯลฯ) ตามพฤติกรรมจริงที่เห็นอีกที
"""

import carla

MIN_GREEN_SEC = 8.0
MAX_GREEN_SEC = 45.0
YELLOW_SEC = 3.0
EMERGENCY_YELLOW_SEC = 1.5   # สั้นกว่าปกติ เพื่อให้ TC-03 ตอบสนองเร็วแต่ยังปลอดภัย
ALL_RED_SEC = 1.0
LOW_VOLUME_THRESHOLD = 2     # รวมรถทั้งแยก <= ค่านี้ ถือว่าเป็นช่วง off-peak (TC-01)

NS_PREFIXES = ("NORTH_INBOUND", "SOUTH_INBOUND")
EW_PREFIXES = ("EAST_INBOUND", "WEST_INBOUND")

# ลำดับ state ของ state machine
GREEN_NS, YELLOW_NS, ALL_RED_1, GREEN_EW, YELLOW_EW, ALL_RED_2 = range(6)


def _group_lanes(lanes_status, prefixes):
    return [l for l in lanes_status if l["lane_id"].startswith(prefixes)]


class TrafficLightGroup:
    """ ห่อรถกลุ่มไฟจราจรของแต่ละทิศ (NS หรือ EW) ให้สั่งสถานะพร้อมกันได้ """

    def __init__(self, traffic_lights):
        self.lights = traffic_lights

    def set_state(self, state: carla.TrafficLightState):
        for tl in self.lights:
            tl.set_state(state)
            tl.freeze(True)  # กัน timer อัตโนมัติของ CARLA มาแย่งควบคุม


def build_light_groups(world, junction):
    """ ดึงไฟจราจรของ junction แล้วแบ่งเป็นกลุ่ม NS / EW คร่าวๆ จากทิศทางที่ไฟหันหน้า
        ควรตรวจสอบด้วยตาจริงในซิมว่าแบ่งถูกฝั่ง ถ้าสลับกันให้สลับ NS_LIGHTS/EW_LIGHTS """
    all_lights = world.get_traffic_lights_in_junction(junction.id)

    ns_lights, ew_lights = [], []
    for tl in all_lights:
        fwd = tl.get_transform().get_forward_vector()
        if abs(fwd.x) > abs(fwd.y):
            ew_lights.append(tl)
        else:
            ns_lights.append(tl)

    print(f"[TL] พบไฟจราจรทั้งหมด {len(all_lights)} ดวง -> NS group {len(ns_lights)} ดวง, "
          f"EW group {len(ew_lights)} ดวง (ควรเช็คด้วยตาว่าถูกฝั่งจริง)")

    return TrafficLightGroup(ns_lights), TrafficLightGroup(ew_lights)


class TrafficController:
    def __init__(self, world, junction):
        self.ns_group, self.ew_group = build_light_groups(world, junction)
        self.state = GREEN_NS
        self.timer = 0.0
        self.current_green_duration = MIN_GREEN_SEC
        self._apply_state()

    # ---------- ส่วนคำนวณเวลาไฟเขียวตาม TC-01 / TC-02 / TC-04 ----------
    def _compute_green_duration(self, lanes_status, favor_group, incident_info):
        ns_lanes = _group_lanes(lanes_status, NS_PREFIXES)
        ew_lanes = _group_lanes(lanes_status, EW_PREFIXES)

        total_ns = sum(l["vehicle_count"] for l in ns_lanes)
        total_ew = sum(l["vehicle_count"] for l in ew_lanes)

        # TC-01: off-peak รถน้อยมากทั้งแยก -> ใช้ไฟเขียวสั้นสุด ลด idle delay
        if total_ns + total_ew <= LOW_VOLUME_THRESHOLD:
            return MIN_GREEN_SEC

        density_ns = sum(l["occupancy_ratio"] for l in ns_lanes)
        density_ew = sum(l["occupancy_ratio"] for l in ew_lanes)

        # TC-04: ถ้าฝั่งไหนติด incident อยู่ ลดสัดส่วนเวลาที่จัดให้ฝั่งนั้นลง
        # (ให้ไฟเขียวเพิ่มไม่ช่วยอะไรเพราะรถขยับไม่ได้ ดันเวลาไปให้อีกฝั่งแทน)
        if incident_info.get("status"):
            blocked_side = "NS" if incident_info["lane_id"].startswith(NS_PREFIXES) else "EW"
            if blocked_side == "NS":
                density_ns *= 0.3
            else:
                density_ew *= 0.3

        total_density = density_ns + density_ew
        my_density = density_ns if favor_group == "NS" else density_ew
        ratio = my_density / total_density if total_density > 0 else 0.5

        # TC-02: จัดสรรตามสัดส่วนความหนาแน่น แต่ครอบด้วย MIN/MAX กันฝั่งรถน้อยถูกทิ้ง
        duration = MIN_GREEN_SEC + ratio * (MAX_GREEN_SEC - MIN_GREEN_SEC)
        return max(MIN_GREEN_SEC, min(MAX_GREEN_SEC, duration))

    def _queue_is_empty(self, lanes_status, prefixes):
        lanes = _group_lanes(lanes_status, prefixes)
        return all(l["vehicle_count"] == 0 for l in lanes) if lanes else True

    # ---------- ส่วนสั่งสถานะไฟจริง ----------
    def _apply_state(self):
        mapping = {
            GREEN_NS:  (carla.TrafficLightState.Green, carla.TrafficLightState.Red),
            YELLOW_NS: (carla.TrafficLightState.Yellow, carla.TrafficLightState.Red),
            ALL_RED_1: (carla.TrafficLightState.Red, carla.TrafficLightState.Red),
            GREEN_EW:  (carla.TrafficLightState.Red, carla.TrafficLightState.Green),
            YELLOW_EW: (carla.TrafficLightState.Red, carla.TrafficLightState.Yellow),
            ALL_RED_2: (carla.TrafficLightState.Red, carla.TrafficLightState.Red),
        }
        ns_state, ew_state = mapping[self.state]
        self.ns_group.set_state(ns_state)
        self.ew_group.set_state(ew_state)

    def _transition(self, new_state):
        self.state = new_state
        self.timer = 0.0
        self._apply_state()

    def get_active_green_group(self):
        """ คืน "NS" / "EW" ถ้าฝั่งนั้นกำลังไฟเขียวอยู่ตอนนี้ หรือ None ถ้าอยู่ช่วงเหลือง/all-red
            ใช้บอก LaneMonitor ว่ารถที่หยุดนิ่งอยู่ในเลนไหน 'ควรขยับได้แล้วแต่ไม่ขยับ' จริงๆ """
        if self.state == GREEN_NS:
            return "NS"
        elif self.state == GREEN_EW:
            return "EW"
        return None

    # ---------- เรียกทุก tick ----------
    def update(self, dt, lanes_status, emergency_info, incident_info):
        self.timer += dt

        # ---- TC-03: emergency preemption ตัดคิวทุกอย่างทันที ----
        if emergency_info.get("emergency_detected"):
            emergency_side = "NS" if emergency_info["lane_id"].startswith(NS_PREFIXES) else "EW"
            currently_green = "NS" if self.state == GREEN_NS else (
                "EW" if self.state == GREEN_EW else None)

            if currently_green != emergency_side:
                print(f"[TC-03] พบรถฉุกเฉินฝั่ง {emergency_side} -> เปิดทาง green wave ทันที")
                if self.state in (GREEN_NS,):
                    self._transition(YELLOW_NS)
                elif self.state in (GREEN_EW,):
                    self._transition(YELLOW_EW)
                # ปล่อยให้ state machine เดินผ่าน yellow/all-red ตามปกติ (สั้นกว่าปกติ) ด้านล่าง

        # ---- state machine หลัก ----
        if self.state == GREEN_NS:
            if self.timer == 0.0 or not hasattr(self, "_ns_target_set"):
                pass
            early_switch = self._queue_is_empty(lanes_status, NS_PREFIXES) and \
                not self._queue_is_empty(lanes_status, EW_PREFIXES)
            if self.timer >= self.current_green_duration or (early_switch and self.timer >= MIN_GREEN_SEC):
                self._transition(YELLOW_NS)

        elif self.state == YELLOW_NS:
            y = EMERGENCY_YELLOW_SEC if emergency_info.get("emergency_detected") else YELLOW_SEC
            if self.timer >= y:
                self._transition(ALL_RED_1)

        elif self.state == ALL_RED_1:
            if self.timer >= ALL_RED_SEC:
                self.current_green_duration = self._compute_green_duration(
                    lanes_status, "EW", incident_info)
                self._transition(GREEN_EW)

        elif self.state == GREEN_EW:
            early_switch = self._queue_is_empty(lanes_status, EW_PREFIXES) and \
                not self._queue_is_empty(lanes_status, NS_PREFIXES)
            if self.timer >= self.current_green_duration or (early_switch and self.timer >= MIN_GREEN_SEC):
                self._transition(YELLOW_EW)

        elif self.state == YELLOW_EW:
            y = EMERGENCY_YELLOW_SEC if emergency_info.get("emergency_detected") else YELLOW_SEC
            if self.timer >= y:
                self._transition(ALL_RED_2)

        elif self.state == ALL_RED_2:
            if self.timer >= ALL_RED_SEC:
                self.current_green_duration = self._compute_green_duration(
                    lanes_status, "NS", incident_info)
                self._transition(GREEN_NS)

        if incident_info.get("status"):
            print(f"[TC-04] ANOMALY ALERT: {incident_info['lane_id']} ติดค้าง "
                  f"{incident_info['stopped_duration_sec']}s -> ลดสัดส่วนไฟเขียวฝั่งนี้ลง")