import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import math
import random
import time
import queue
import numpy as np
import cv2
import open3d as o3d
from pascal_voc_writer import Writer


def add_open3d_axis(vis):
    axis = o3d.geometry.LineSet()
    axis.points = o3d.utility.Vector3dVector(np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ]))
    axis.lines = o3d.utility.Vector2iVector(np.array([
        [0, 1],
        [0, 2],
        [0, 3]
    ]))
    axis.colors = o3d.utility.Vector3dVector(np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]
    ]))
    vis.add_geometry(axis)
    
# Camera projection matrix to get the 2D bounding box of a 3D object
def build_projection_matrix(w, h, fov, is_behind_camera=False):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    if is_behind_camera:
        K[0, 0] = K[1, 1] = -focal
    else:
        K[0, 0] = K[1, 1] = focal

    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K

def get_image_point(loc, K, w2c):
        # Calculate 2D projection of 3D coordinate

        # Format the input coordinate (loc is a carla.Position object)
        point = np.array([loc.x, loc.y, loc.z, 1])
        # transform to camera coordinates
        point_camera = np.dot(w2c, point)

        # New we must change from UE4's coordinate system to an "standard"
        # (x, y ,z) -> (y, -z, x)
        # and we remove the fourth componebonent also
        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        # now project 3D->2D using the camera matrix
        point_img = np.dot(K, point_camera)
        # normalize
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]

def create_directories(base_path, rain_level):
    camera_dir = os.path.join(base_path, "camera", str(int(rain_level)))
    lidar_dir = os.path.join(base_path, "lidar", str(int(rain_level)))
    calyo_dir = os.path.join(base_path, "calyosensor", str(int(rain_level)))
    os.makedirs(camera_dir, exist_ok=True)
    os.makedirs(lidar_dir, exist_ok=True)
    os.makedirs(calyo_dir, exist_ok=True)
    return camera_dir, lidar_dir, calyo_dir

def save_calyo_ply(calyo_data, calyo_path):
    points = []
    intensities = []
    
    for detection in calyo_data:
        x = detection.depth * math.cos(detection.altitude) * math.cos(detection.azimuth)
        y = detection.depth * math.cos(detection.altitude) * math.sin(detection.azimuth)
        z = detection.depth * math.sin(detection.altitude)
        points.append([x, y, z])
        intensities.append(detection.intensity)
    
    # Convert lists to numpy arrays
    points = np.array(points)
    intensities = np.array(intensities)

    # Normalize intensities to use as colors
    intensities_normalized = intensities / intensities.max()
    colors = np.tile(intensities_normalized[:, None], (1, 3))  # Same intensity for R, G, B

    # Create Open3D point cloud object
    calyo_points = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    calyo_points.colors = o3d.utility.Vector3dVector(colors)

    # Save to .ply file
    o3d.io.write_point_cloud(calyo_path, calyo_points)

# Compute bounding box center, dimensions, and yaw in sensor coordinates
def compute_bounding_box_in_sensor(vertices):
    # Compute the center of the bounding box in sensor coordinates
    x_center = np.mean([v[0] for v in vertices])
    y_center = np.mean([v[1] for v in vertices])
    z_center = np.mean([v[2] for v in vertices])

    # Compute dimensions (dx, dy, dz) in sensor coordinates
    dx = np.max([v[0] for v in vertices]) - np.min([v[0] for v in vertices])
    dy = np.max([v[1] for v in vertices]) - np.min([v[1] for v in vertices])
    dz = np.max([v[2] for v in vertices]) - np.min([v[2] for v in vertices])

    # Compute the yaw angle (assuming the vehicle is mostly aligned with the x-axis)
    yaw = np.arctan2(vertices[1][1] - vertices[0][1], vertices[1][0] - vertices[0][0])

    category_name = 'vehicle'

    return [x_center, y_center, z_center, dx, dy, dz, yaw, category_name]

client = carla.Client('localhost', 2000)
world  = client.get_world()
bp_lib = world.get_blueprint_library()

# Get the map spawn points
spawn_points = world.get_map().get_spawn_points()

# spawn vehicle
vehicle_bp =bp_lib.find('vehicle.lincoln.mkz_2020')
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

# spawn camera
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_init_trans = carla.Transform(carla.Location(z=2))
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
vehicle.set_autopilot(True)

# Set up the simulator in synchronous mode
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

# Create a queue to store and retrieve the sensor data
image_queue = queue.Queue()
camera.listen(image_queue.put)

# Get the world to camera matrix
world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

# Get the attributes from the camera
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()
fov = camera_bp.get_attribute("fov").as_float()

# Calculate the camera projection matrix to project from 3D -> 2D
K = build_projection_matrix(image_w, image_h, fov)
K_b = build_projection_matrix(image_w, image_h, fov, is_behind_camera=True)
edges = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]

for i in range(50):
    vehicle_bp = random.choice(bp_lib.filter('vehicle'))
    npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
    if npc:
        npc.set_autopilot(True)

# Retrieve the first image
world.tick()
image = image_queue.get()

# Reshape the raw data into an RGB array
img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4)) 

# Display the image in an OpenCV display window
cv2.namedWindow('ImageWindowName', cv2.WINDOW_AUTOSIZE)
cv2.imshow('ImageWindowName',img)
cv2.waitKey(1)

def point_in_canvas(pos, img_h, img_w):
    """Return true if point is in canvas"""
    if (pos[0] >= 0) and (pos[0] < img_w) and (pos[1] >= 0) and (pos[1] < img_h):
        return True
    return False

while True:
    # Retrieve and reshape the image
    world.tick()
    image = image_queue.get()

    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

    # Get the camera matrix 
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())
    
    frame_path = 'output/%06d' % image.frame

    # Save the image
    image.save_to_disk(frame_path + '.png')
    
    # # Process and save the lidar point cloud with intensity
    # lidar_data = np.copy(np.frombuffer(lidar_point_cloud.raw_data, dtype=np.float32))
    # lidar_data = lidar_data.reshape([-1, 4])  # x, y, z, intensity

    # # Normalize intensity and repeat it across three channels (R, G, B)
    # intensity = lidar_data[:, 3] / 255.0
    # colors = np.tile(intensity[:, None], (1, 3))  # Create an array with shape (N, 3) where N is the number of points

    # # Create point cloud object
    # lidar_points = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(lidar_data[:, :3]))
    # lidar_points.colors = o3d.utility.Vector3dVector(colors)

    # # Save the point cloud to a .ply file
    # o3d.io.write_point_cloud(lidar_path, lidar_points)

    # # Process and save CalyoSensor data as a point cloud
    # save_calyo_ply(calyo_data, calyo_path)
    
    # Initialize the exporter
    writer = Writer(frame_path + '.png', image_w, image_h)
    
    for npc in world.get_actors().filter('*vehicle*'):

        # Filter out the ego vehicle
        if npc.id != vehicle.id:

            bb = npc.bounding_box
            dist = npc.get_transform().location.distance(vehicle.get_transform().location)

            # Filter for the vehicles within 50m
            if dist < 50:

            # Calculate the dot product between the forward vector
            # of the vehicle and the vector between the vehicle
            # and the other vehicle. We threshold this dot product
            # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
                forward_vec = vehicle.get_transform().get_forward_vector()
                ray = npc.get_transform().location - vehicle.get_transform().location
                
                #     ## 3D Bounding Box
                # if forward_vec.dot(ray) > 0:
                #     verts = [v for v in bb.get_world_vertices(npc.get_transform())]
                #     for edge in edges:
                #         p1 = get_image_point(verts[edge[0]], K, world_2_camera)
                #         p2 = get_image_point(verts[edge[1]],  K, world_2_camera)

                #         p1_in_canvas = point_in_canvas(p1, image_h, image_w)
                #         p2_in_canvas = point_in_canvas(p2, image_h, image_w)

                #         if not p1_in_canvas and not p2_in_canvas:
                #             continue

                #         ray0 = verts[edge[0]] - camera.get_transform().location
                #         ray1 = verts[edge[1]] - camera.get_transform().location
                #         cam_forward_vec = camera.get_transform().get_forward_vector()

                #         # One of the vertex is behind the camera
                #         if not (cam_forward_vec.dot(ray0) > 0):
                #             p1 = get_image_point(verts[edge[0]], K_b, world_2_camera)
                #         if not (cam_forward_vec.dot(ray1) > 0):
                #             p2 = get_image_point(verts[edge[1]], K_b, world_2_camera)

                #         cv2.line(img, (int(p1[0]),int(p1[1])), (int(p2[0]),int(p2[1])), (255,0,0, 255), 1)        
                
                ## 2D Bounding Box
                if forward_vec.dot(ray) > 0:
                    p1 = get_image_point(bb.location, K, world_2_camera)
                    verts = [v for v in bb.get_world_vertices(npc.get_transform())]
                    x_max = -10000
                    x_min = 10000
                    y_max = -10000
                    y_min = 10000

                    for vert in verts:
                        p = get_image_point(vert, K, world_2_camera)
                        # Find the rightmost vertex
                        if p[0] > x_max:
                            x_max = p[0]
                        # Find the leftmost vertex
                        if p[0] < x_min:
                            x_min = p[0]
                        # Find the highest vertex
                        if p[1] > y_max:
                            y_max = p[1]
                        # Find the lowest  vertex
                        if p[1] < y_min:
                            y_min = p[1]
                    
                    # # Add the object to the frame (ensure it is inside the image)
                    # if x_min > 0 and x_max < image_w and y_min > 0 and y_max < image_h: 
                    #     writer.addObject('vehicle', x_min, y_min, x_max, y_max)
                    
                    cv2.line(img, (int(x_min),int(y_min)), (int(x_max),int(y_min)), (0,0,255, 255), 1)
                    cv2.line(img, (int(x_min),int(y_max)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
                    cv2.line(img, (int(x_min),int(y_min)), (int(x_min),int(y_max)), (0,0,255, 255), 1)
                    cv2.line(img, (int(x_max),int(y_min)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
                    
                    # Add the object to the frame (ensure it is inside the image)
                    if x_min > 0 and x_max < image_w and y_min > 0 and y_max < image_h: 
                        writer.addObject('vehicle', x_min, y_min, x_max, y_max)
    
    # Save the bounding boxes in the scene
    writer.save(frame_path + '.xml')
    
    cv2.imshow('ImageWindowName',img)
    if cv2.waitKey(1) == ord('q'):
        break
cv2.destroyAllWindows()