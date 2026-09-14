import carla
import cv2
import numpy as np
import random
import time
import json

from vision_analytics import LaneMonitor
from traffic_controller import TrafficController

# --- CONFIGURATION ---
CARLA_HOST = 'localhost'
CARLA_PORT = 2000
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
CAMERA_FOV = 90

current_image = None

def process_camera_image(image):
    global current_image
    raw_array = np.frombuffer(image.raw_data, dtype=np.uint8)
    bgra_image = raw_array.reshape((image.height, image.width, 4))
    bgr_image = bgra_image[:, :, :3]
    current_image = bgr_image

def apply_weather_scenario(world, weather_id="ENV-01"):
    """ ฟังก์ชันตั้งค่าสภาพอากาศตาม Matrix (ENV-01 ถึง ENV-04) """
    weather = carla.WeatherParameters()
    
    if weather_id == "ENV-01":  # Clear Daylight
        weather.cloudiness = 10.0
        weather.precipitation = 0.0
        weather.sun_altitude_angle = 75.0
        print("[SCENARIO] Applied ENV-01: Clear Daylight")
    elif weather_id == "ENV-02":  # Night Lights
        weather.cloudiness = 20.0
        weather.precipitation = 0.0
        weather.sun_altitude_angle = -30.0  # กลางคืน
        print("[SCENARIO] Applied ENV-02: Night Lights")
    elif weather_id == "ENV-03":  # Heavy Rain
        weather.cloudiness = 90.0
        weather.precipitation = 80.0
        weather.precipitation_deposits = 90.0
        weather.sun_altitude_angle = 30.0
        print("[SCENARIO] Applied ENV-03: Heavy Rain")
    elif weather_id == "ENV-04":  # Dense Fog
        weather.cloudiness = 80.0
        weather.fog_density = 75.0
        weather.fog_distance = 10.0
        print("[SCENARIO] Applied ENV-04: Dense Fog")
        
    world.set_weather(weather)

def spawn_traffic(world, client, num_vehicles=30):
    """ ฟังก์ชันสุ่มสร้างรถวิ่งในเมืองตามจำนวนที่กำหนด """
    blueprint_library = world.get_blueprint_library()

    # รถคันใหญ่ (รถ 10 ล้อ/บรรทุก/บัส) เลี้ยวไม่เต็มวงในทางแยกแคบของ Town01
    # ทำให้ค้างขวางเลนจนรถคันอื่นติดตามไปด้วย เลยกันออกจากกลุ่ม "รถพื้นหลัง" ทั่วไป
    # หมายเหตุ: ไม่ได้ตัด firetruck/ambulance ทิ้งถาวร แค่ไม่ให้ถูกสุ่มมาเป็นรถพื้นหลัง
    # ตอนทำ TC-03 (Emergency Vehicle Preemption) ค่อย spawn รถฉุกเฉินแยกต่างหากเองได้ตามปกติ
    EXCLUDED_LARGE_VEHICLES = [
        'vehicle.carlamotors.firetruck',
        'vehicle.carlamotors.carlacola',
        'vehicle.carlamotors.european_hgv',
        'vehicle.mitsubishi.fusorosa',   # รถบัส
        'vehicle.mercedes.sprinter',     # แวนคันใหญ่
        'vehicle.ford.ambulance',
        'vehicle.volkswagen.t2',
        'vehicle.volkswagen.t2_2021',
    ]

    all_vehicle_blueprints = blueprint_library.filter('vehicle.*')
    vehicle_blueprints = [
        bp for bp in all_vehicle_blueprints
        if bp.id not in EXCLUDED_LARGE_VEHICLES
    ]
    print(f"[INFO] กรองรถคันใหญ่ออก {len(all_vehicle_blueprints) - len(vehicle_blueprints)} "
          f"แบบ เหลือ blueprint รถพื้นหลัง {len(vehicle_blueprints)} แบบ")

    spawn_points = world.get_map().get_spawn_points()
    
    # เปิดการทำงานของ Traffic Manager
    traffic_manager = client.get_trafficmanager()
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)
    traffic_manager.set_synchronous_mode(False)
    
    spawned_vehicles = []
    random.shuffle(spawn_points)

    if len(spawn_points) < num_vehicles:
        print(f"[WARNING] Town01 มี spawn point แค่ {len(spawn_points)} จุด "
              f"แต่ขอรถ {num_vehicles} คัน จะได้รถแค่เท่าที่จุดมีพอ")
    
    for point in spawn_points[:num_vehicles]:
        bp = random.choice(vehicle_blueprints)
        # ปรับสีสุ่มเพื่อทดสอบระบบ Vision
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)
            
        vehicle = world.try_spawn_actor(bp, point)
        if vehicle is not None:
            vehicle.set_autopilot(True, traffic_manager.get_port())
            spawned_vehicles.append(vehicle)
            time.sleep(0.05)  # เว้นจังหวะเล็กน้อยระหว่าง spawn กันฟิสิกส์ชนกันตอนเกิดพร้อมกันเป๊ะ
            
    print(f"[TRAFFIC] Spawned {len(spawned_vehicles)} vehicles driven by Autopilot.")
    return spawned_vehicles

def get_junction_by_id(world, junction_id):
    """ หา junction object จาก id ที่รู้ล่วงหน้า (306 = สี่แยกที่เลือกใช้จาก find_intersection.py) """
    topology = world.get_map().get_topology()
    for wp_pair in topology:
        for wp in wp_pair:
            if wp.is_junction and wp.get_junction().id == junction_id:
                return wp.get_junction()
    return None


def main():
    global current_image
    actor_list = []

    try:
        print("[INFO] Connecting to CARLA Server...")
        client = carla.Client(CARLA_HOST, CARLA_PORT)
        client.set_timeout(10.0)

        world = client.get_world()
        if 'Town01' not in world.get_map().name:
            print("[INFO] Loading Town01 map...")
            world = client.load_world('Town01')

        # สำคัญมาก: บังคับ world settings ให้เป็น asynchronous ตั้งแต่ต้น
        # เพราะถ้า world เคยถูกตั้งเป็น synchronous_mode=True จากสคริปต์อื่นก่อนหน้า
        # ค่านี้จะค้างอยู่บนฝั่ง server ข้ามการรัน client ใหม่ ถ้าไม่มีใครเรียก world.tick()
        # ซิมจะไม่ขยับเลยแม้แต่เฟรมเดียว ทำให้รถค้างนิ่งเหมือน "พัง" ตอน spawn/เลี้ยว
        settings = world.get_settings()
        if settings.synchronous_mode:
            print("[INFO] Detected leftover synchronous_mode=True, resetting to asynchronous...")
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        # สำคัญ: เคลียร์รถที่ตกค้างจากการรันสคริปต์รอบก่อนๆ ทิ้งก่อนเสมอ
        # เพราะถ้าปิดโปรแกรมด้วย Ctrl+C หรือปิดหน้าต่างตรงๆ (ไม่ได้กด 'q')
        # โค้ดใน finally: ที่ทำลาย actor จะไม่ถูกเรียก รถเก่าจะค้างอยู่ในโลกไปเรื่อยๆ
        # พอ spawn รถชุดใหม่ซ้อนทับ จะชนกันเองจนอุดตันสี่แยกสะสมทุกรอบที่รัน
        leftover_vehicles = world.get_actors().filter('vehicle.*')
        if len(leftover_vehicles) > 0:
            print(f"[INFO] พบรถตกค้างจากรอบก่อน {len(leftover_vehicles)} คัน กำลังลบทิ้ง...")
            for v in leftover_vehicles:
                v.destroy()

        # 1. ปรับสภาพอากาศ (ทดลองเปลี่ยนเป็น ENV-01, ENV-02, ENV-03, ENV-04 ได้ที่นี่)
        apply_weather_scenario(world, weather_id="ENV-01")

        # 2. ปล่อยรถจำลองวิ่งรอบทางแยก (ตั้งค่าจำนวนรถตาม TC-01 ถึง TC-04)
        vehicles = spawn_traffic(world, client, num_vehicles=50)
        actor_list.extend(vehicles)

        blueprint_library = world.get_blueprint_library()

        # 3. เซ็ตตำแหน่งกล้อง CCTV ส่องทางแยก
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        camera_bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        camera_bp.set_attribute('fov', str(CAMERA_FOV))

        # Junction 306 (x≈94.07, y≈131.03) — ยืนยันซ้ำด้วย waypoint แล้วตรงกับรอบก่อนหน้า
        # แปลว่าจุดไม่ได้ผิด แต่การเยื้องกล้องออกด้านข้าง (SW) พาไปอยู่ฝั่งที่มีตึก/สวนบัง
        # แก้โดยเปลี่ยนเป็นกล้องมองตั้งฉากจากด้านบน (bird's-eye) แทนการเยื้องข้าง
        # เพราะมองจากด้านบนตรงๆ จะไม่มีตึกฝั่งไหนมาบังมุมมองได้ ตราบใดที่สูงกว่าตึก
        target_x, target_y = 94.07, 131.03

        CAM_HEIGHT = 70.0    # สูงพอที่จะพ้นตึก 3 ชั้น (~12-15m) ไปมาก
        CAM_PITCH = -85.0    # เกือบตั้งฉากลงพื้น ไม่ใช่มุมเฉียงแบบเดิม

        cam_x = target_x
        cam_y = target_y
        cam_z = CAM_HEIGHT
        yaw = 0.0  # มองตรงลงมา แทบไม่มีผลต่อองศาที่เห็น เพราะเกือบตั้งฉากแล้ว

        camera_transform = carla.Transform(
            carla.Location(x=cam_x, y=cam_y, z=cam_z),
            carla.Rotation(pitch=CAM_PITCH, yaw=yaw, roll=0.0)
        )

        camera = world.spawn_actor(camera_bp, camera_transform)
        actor_list.append(camera)
        print("[INFO] Virtual CCTV Camera spawned successfully.")

        camera.listen(lambda image: process_camera_image(image))

        # 4. ผูก Vision Analytics + Traffic Controller เข้ากับสี่แยก (Junction 306)
        junction = get_junction_by_id(world, 306)
        if junction is None:
            raise RuntimeError("หา Junction 306 ไม่เจอ — ตรวจสอบว่า id ยังตรงกับที่ find_intersection.py รายงานไว้")

        lane_monitor = LaneMonitor(world, junction, intersection_id="INT-CARLA-TOWN01")
        controller = TrafficController(world, junction)
        print("[INFO] Lane monitor + traffic controller (TC-01~TC-04) พร้อมทำงานแล้ว")

        print("[INFO] Simulation active. Press 'q' on the OpenCV window to exit.")

        UPDATE_INTERVAL = 0.5  # วิเคราะห์และตัดสินใจไฟทุก 0.5 วินาที
        last_update_time = time.time()
        last_payload = None

        while True:
            now = time.time()

            if now - last_update_time >= UPDATE_INTERVAL:
                dt = now - last_update_time
                lanes_status, emergency_info, incident_info = lane_monitor.update()
                controller.update(dt, lanes_status, emergency_info, incident_info)

                last_payload = lane_monitor.build_mqtt_payload()
                # ยังไม่ส่ง MQTT จริง (ทำวันที่ 18-19 ก.ย.) แค่ print ไว้ดูโครงสร้างก่อน
                print(json.dumps(last_payload, ensure_ascii=False))

                last_update_time = now

            if current_image is not None:
                display_frame = current_image.copy()
                if last_payload is not None:
                    y = 20
                    for lane in last_payload["lanes_status"]:
                        text = (f"{lane['lane_id']}: n={lane['vehicle_count']} "
                                f"occ={lane['occupancy_ratio']} v={lane['avg_speed_kmh']}km/h")
                        cv2.putText(display_frame, text, (10, y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
                        y += 18
                    if last_payload["emergency_detected"]:
                        cv2.putText(display_frame, "EMERGENCY PREEMPTION ACTIVE", (10, y + 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
                    if last_payload["incident_detected"]["status"]:
                        cv2.putText(display_frame, "INCIDENT DETECTED", (10, y + 35),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 140, 255), 2, cv2.LINE_AA)
                cv2.imshow("Smart Traffic - CARLA Intersection CCTV", display_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"[ERROR] An exception occurred: {e}")

    finally:
        print("[INFO] Cleaning up spawned actors and vehicles...")
        for actor in actor_list:
            actor.destroy()
        cv2.destroyAllWindows()
        print("[INFO] Clean up complete.")

if __name__ == '__main__':
    main()