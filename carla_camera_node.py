import carla
import cv2
import numpy as np
import time

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

        blueprint_library = world.get_blueprint_library()

        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(IMAGE_WIDTH))
        camera_bp.set_attribute('image_size_y', str(IMAGE_HEIGHT))
        camera_bp.set_attribute('fov', str(CAMERA_FOV))

        camera_transform = carla.Transform(
            carla.Location(x=154.0, y=190.0, z=7.0),
            carla.Rotation(pitch=-30.0, yaw=-90.0, roll=0.0)
        )

        camera = world.spawn_actor(camera_bp, camera_transform)
        actor_list.append(camera)
        print("[INFO] Virtual CCTV Camera spawned successfully.")

        camera.listen(lambda image: process_camera_image(image))
        print("[INFO] Stream started. Press 'q' on the OpenCV window to exit.")

        while True:
            if current_image is not None:
                cv2.imshow("Smart Traffic - CARLA Intersection CCTV", current_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        print(f"[ERROR] An exception occurred: {e}")

    finally:
        print("[INFO] Cleaning up spawned actors...")
        for actor in actor_list:
            actor.destroy()
        cv2.destroyAllWindows()
        print("[INFO] Process finished successfully.")

if __name__ == '__main__':
    main()