"""
สคริปต์ช่วยหาตำแหน่งสี่แยก (junction) จริงใน Town01
รันแยกจาก carla_camera_node.py เพื่อดูพิกัดที่ถูกต้องก่อน

ผลลัพธ์ที่ใช้แล้ว: เลือก Junction 306 (x=92.76, y=130.84) เป็นสี่แยกหลัก
เพราะอยู่บนถนนสาย N-S ที่มีทางแยกต่อเนื่อง (x~92) ตรงกับ TC-02
"""
import carla

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    world = client.get_world()
    if 'Town01' not in world.get_map().name:
        world = client.load_world('Town01')

    carla_map = world.get_map()
    topology = carla_map.get_topology()

    # ดึง waypoint ที่เป็นจุดตัด (junction) ทั้งหมด แล้ว dedupe ด้วย junction id
    junctions = {}
    for wp_pair in topology:
        for wp in wp_pair:
            if wp.is_junction:
                j = wp.get_junction()
                junctions[j.id] = j

    print(f"[INFO] พบ junction ทั้งหมด {len(junctions)} จุดใน Town01\n")

    for jid, junction in junctions.items():
        # bounding_box.location ไม่แม่น เพราะรวมพื้นที่ทางเท้า/เกาะกลางที่ไม่สมมาตร
        # ทำให้จุดศูนย์กลางหลุดออกไปนอกผิวถนนได้ (เช่นไปตกกลางสวนสาธารณะ)
        # ใช้ waypoint จริงบนผิวถนนที่วิ่งผ่านทางแยกแทน แล้วเฉลี่ยตำแหน่งจะแม่นกว่ามาก
        lane_waypoints = junction.get_waypoints(carla.LaneType.Driving)

        if not lane_waypoints:
            continue

        xs, ys, zs = [], [], []
        for wp_enter, wp_exit in lane_waypoints:
            for wp in (wp_enter, wp_exit):
                xs.append(wp.transform.location.x)
                ys.append(wp.transform.location.y)
                zs.append(wp.transform.location.z)

        center = carla.Location(
            x=sum(xs) / len(xs),
            y=sum(ys) / len(ys),
            z=sum(zs) / len(zs)
        )

        # เช็คว่าเป็นทางแยก 4 ทางจริงหรือแค่ 3 ทาง (T-junction)
        # นับจากจำนวนไฟจราจรที่ควบคุม junction นี้ — 4 ทางปกติจะมีไฟ >= 4 ดวง
        num_lights = len(world.get_traffic_lights_in_junction(jid))
        shape_note = "4-way จริง" if num_lights >= 4 else f"ระวัง! อาจเป็น T-junction (พบไฟแค่ {num_lights} ดวง)"

        print(f"Junction ID {jid}: center (จาก {len(lane_waypoints)} เลน, ไฟจราจร {num_lights} ดวง) = "
              f"(x={center.x:.2f}, y={center.y:.2f}, z={center.z:.2f}) -> {shape_note}")

        # วาดจุดสีแดงค้างไว้ 60 วิ ให้เห็นใน simulator ว่าคือจุดไหน
        world.debug.draw_point(
            center + carla.Location(z=2.0),
            size=0.3,
            color=carla.Color(255, 0, 0),
            life_time=60.0
        )
        world.debug.draw_string(
            center + carla.Location(z=3.0),
            f"J{jid}",
            color=carla.Color(255, 255, 0),
            life_time=60.0
        )

    print("\n[INFO] เปิดหน้าต่าง CARLA Spectator ดูได้เลยว่าจุดสีแดง/ตัวเลขสีเหลืองอยู่ตรงไหน")
    print("[INFO] เลือก junction ที่ต้องการ แล้วเอาพิกัด x, y มาใช้แทนค่าที่ hardcode ไว้")

if __name__ == '__main__':
    main()