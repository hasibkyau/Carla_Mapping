import glob
import os
import sys
import time
import numpy as np
import open3d as o3d
from csv import writer

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

actor_list = []

def preProcess(image):
    image.save_to_disk('hybrid_dataset/lidar/%06d.ply' % image.frame_number)

    location = vehicle.get_location()
    arr = [image.frame_number,location.x,location.y,location.z]
    with open('hybrid_dataset/vehicle_location.csv', 'a') as f_object:
        writer_object = writer(f_object)
        writer_object.writerow(arr)
        f_object.close()
    print(type(location.x))

    c = vehicle.get_control()
    arr2 = [image.frame_number, c.throttle, c.steer, c.brake, c.gear]
    with open('hybrid_dataset/driving_log.csv', 'a') as f_object:
        writer_object = writer(f_object)
        writer_object.writerow(arr2)
        f_object.close()



try:
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)

    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    bp = blueprint_library.filter("model3")[0]
    print(bp)

    list = world.get_map().get_spawn_points()
    print(list)
    spawn_point = list[4]
    # spawn_point = random.choice(world.get_map().get_spawn_points())

    vehicle = world.spawn_actor(bp, spawn_point)
    vehicle.set_autopilot(True)
    # vehicle.apply_control(carla.VehicleControl(throttle = 0.0, steer = 0.0))
    actor_list.append(vehicle)

    lidar_bp = blueprint_library.find("sensor.lidar.ray_cast")
    lidar_bp.set_attribute('range', '5000') #Maximum distance to measure/raycast in centimeter
    # lidar_bp.set_attribute('upper_fov', '60') #Angle in degrees of the highest laser.
    # lidar_bp.set_attribute('channels', '100') #Number of lasers.

    cam_bp = blueprint_library.find("sensor.camera.rgb")
    cam_bp.set_attribute('image_size_x', f'{800}')
    cam_bp.set_attribute('image_size_y', f'{600}')
    cam_bp.set_attribute('fov', '110')

    segmantic_bp = blueprint_library.find("sensor.camera.semantic_segmentation")
    obstacle_bp = blueprint_library.find("sensor.other.obstacle")
    collision_bp = blueprint_library.find("sensor.other.collision")

    spawn_point = carla.Transform(carla.Location(x=2.5, z=2))

    lidar_sensor = world.spawn_actor(lidar_bp, spawn_point, attach_to=vehicle)
    actor_list.append(lidar_sensor)

    camera_sensor = world.spawn_actor(cam_bp, spawn_point, attach_to=vehicle)
    actor_list.append(camera_sensor)

    semantic_sensor = world.spawn_actor(segmantic_bp, spawn_point, attach_to=vehicle)
    actor_list.append(semantic_sensor)

    lidar_sensor.listen(lambda image: preProcess(image))
    camera_sensor.listen(lambda image: image.save_to_disk('hybrid_dataset/rgb_camera/%06d.png' % image.frame_number))
    semantic_sensor.listen(lambda image: image.save_to_disk('hybrid_dataset/semantic_camera/%06d.png' % image.frame_number))

    # sensor.listen(lambda image: image.save_to_disk('data.ply'))

    location = vehicle.get_location()
    print(location)

    # while True:
    #     location = vehicle.get_location()
    #     # print(location)

    """
    # create visualizer and window.
    vis = o3d.visualization.Visualizer()
    vis.create_window(height=480, width=640)

    # initialize pointcloud instance.
    pcd = o3d.geometry.PointCloud()

    # *optionally* add initial points
    points = np.random.rand(10, 3)
    pcd.points = o3d.utility.Vector3dVector(points)

    # include it in the visualizer before non-blocking visualization.
    vis.add_geometry(pcd)

    # to add new points each dt secs.
    dt = 0.01
    previous_t = time.time()

    # To exit, press 'q' or click the 'x' of the window.
    keep_running = True

    while keep_running:
        if time.time() - previous_t > dt:

            cloud = (o3d.io.read_point_cloud("data.ply"))
            # pcd.points = cloud.points
            pcd.points.extend(np.asarray(cloud.points))
            vis.update_geometry(pcd)
            previous_t = time.time()

        keep_running = vis.poll_events()
        vis.update_renderer()
    vis.destroy_window()
    """

    time.sleep(1200)
finally:
    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")








