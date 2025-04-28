#!/usr/bin/env python

# Author: Oscar Siu (oscarsiu21@gmail.com)
# Student number: 2441322
# Last Modified: 2025-02-25
# Version: 5.0
# This work is created for MSc Robotics Dissertation Project at 
# University of Bristol and University of West of England

# Description: Simulate an ego vehicle equipped with sensors (camera, radar) and visualize sensor data on pygame window.
# This scripts records data collected and analyze scattering effect due to rain.

import glob
import os
import sys

'''
# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
'''
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    print("carla library not found.")
    sys.exit(1)

import carla
import random
import time
import pygame
import numpy as np
import queue
import matplotlib.pyplot as plt
import math
import cv2
import open3d as o3d
from matplotlib import cm
import pandas as pd
import argparse
from pascal_voc_writer import Writer
from pygame.locals import K_ESCAPE, K_q

# Camera projection matrix
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
        # and we remove the fourth component also
        point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

        # now project 3D->2D using the camera matrix
        point_img = np.dot(K, point_camera)
        # normalize
        point_img[0] /= point_img[2]
        point_img[1] /= point_img[2]

        return point_img[0:2]

'''
# ==============================================================================
# -- Initialization ---------------------------------------------------------
# ==============================================================================
'''

rain_intensity = 0.0
fog_density = 10.0

PARENT_FOLDER = "straightline_fog_10"

# Create parent and subfolders
os.makedirs(PARENT_FOLDER, exist_ok=True)
os.makedirs(os.path.join(PARENT_FOLDER, "images"), exist_ok=True)
os.makedirs(os.path.join(PARENT_FOLDER, "pointcloud"), exist_ok=True)
os.makedirs(os.path.join(PARENT_FOLDER, "ground_truth"), exist_ok=True)


# Set rainy weather
def set_weather(world):
    weather = carla.WeatherParameters(
        sun_altitude_angle=45.0, # Day: 45.0, Night: -10, Sunset: 0.5
        sun_azimuth_angle=0.0,
        cloudiness=50.0,
        precipitation=rain_intensity, precipitation_deposits=rain_intensity,  # Rain intensity
        wind_intensity=20.0,
        fog_density=fog_density, fog_distance=0.75, fog_falloff=0.01 * fog_density /2,
        wetness=rain_intensity,
        scattering_intensity=0.0, mie_scattering_scale=0.03, rayleigh_scattering_scale=0.0331,
    )
    world.set_weather(weather)
    print('Set weather: ', weather)

'''
# ==============================================================================
# -- Actors --------------------------------------------------------------------
# ==============================================================================
'''
# Spawn ego vehicle
def spawn_ego_vehicle(world):
    ego_bp = bp_lib.find('vehicle.toyota.prius')
    ego_bp.set_attribute('role_name', 'hero')
    ego_bp.set_attribute('color', '85,229,167')

    ego_spawnpoint = world.get_map().get_spawn_points()[20]
    vehicle = world.spawn_actor(ego_bp, ego_spawnpoint)
    print('Spawned ego vehicle')

    # Get the bounding box of the vehicle
    bounding_box = vehicle.bounding_box
    dimensions = bounding_box.extent  # Extent gives half-lengths in x, y, z

    # Calculate full dimensions
    length = 2 * dimensions.x
    width = 2 * dimensions.y
    height = 2 * dimensions.z

    print(f"Vehicle Dimensions: Length={length}m, Width={width}m, Height={height}m")
    return vehicle, ego_spawnpoint

# Spawn NPC vehicles
def spawn_npcs(world, vehicle_spawnpoint):
    #npc_sp = [148, 106, 104, 20, 18]      
    npc_sp = [186, 181, 184, 19, 18]      
    npc_bp = random.choice(bp_lib.filter('vehicle.*'))
    v1 = world.spawn_actor(npc_bp, carla.Transform(carla.Location(
        vehicle_spawnpoint.location.x+50, vehicle_spawnpoint.location.y-3, 
        vehicle_spawnpoint.location.z)))

    v2 = world.spawn_actor(npc_bp, carla.Transform(carla.Location(
        vehicle_spawnpoint.location.x+110, vehicle_spawnpoint.location.y, 
        vehicle_spawnpoint.location.z)))
        
    v3 = world.spawn_actor(npc_bp, carla.Transform(carla.Location(
        vehicle_spawnpoint.location.x+30, vehicle_spawnpoint.location.y+3, 
        vehicle_spawnpoint.location.z)))
                
        #print(f'Spawned {len(npc_sp)} vehicles')

'''
# ==============================================================================
# -- Sensor configuration-------------------------------------------------------
# ==============================================================================
'''
# Define sensors attached to ego vehicle
def attach_sensors(world, vehicle):
    # Camera
    camera_bp = bp_lib.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '1280')
    camera_bp.set_attribute('image_size_y', '720')
    camera_bp.set_attribute('fov', '120')
    
    camera_transform = carla.Transform(carla.Location(x=-5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    # Radar
    radar_bp = bp_lib.find('sensor.other.calyopulse')
    radar_bp.set_attribute('horizontal_fov', '70')
    radar_bp.set_attribute('vertical_fov', '55')
    radar_bp.set_attribute('range', '30')
    radar_bp.set_attribute('points_per_second', '80000')
    
    radar_transform = carla.Transform(carla.Location(x=2.0, z=1.0))
    radar = world.spawn_actor(radar_bp, radar_transform, attach_to=vehicle)
    
    ## Lidar
    # lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
    # lidar_bp.set_attribute('channels', '32')
    # lidar_bp.set_attribute('points_per_second',str(100000))
    # lidar_bp.set_attribute('rotation_frequency',str(10.0))
    # lidar_bp.set_attribute('upper_fov',str(30.0))
    # lidar_bp.set_attribute('lower_fov',str(-25.0))
    # lidar_bp.set_attribute('horizontal_fov',str(360))
    # lidar_bp.set_attribute('range','50')

    # lidar_transform = carla.Transform(carla.Location(x=0, z=2.4))
    # lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)

    return camera, radar  

def process_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
    array = array[:, :, :3][:, :, ::-1] # Keep RGB part
    return array

def rad_callback(radar_data, world):
    radar_points = []
    intensities = []
    # Process and visualize radar data
    # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
    # points = np.reshape(points, (len(radar_data), 4))
    
    velocity_range = 7.5 # m/s
    current_rot = radar_data.transform.rotation
    height_threshold = -0.03
    
    sensor_transform = radar_data.transform  # Get radar sensor transform (position & orientation)  
    
            
    for detect in radar_data:
        azi = math.degrees(detect.azimuth)
        alt = math.degrees(detect.altitude)
        x = detect.depth * np.cos(detect.azimuth) * np.cos(detect.altitude)
        y = detect.depth * np.sin(detect.azimuth) * np.cos(detect.altitude)
        z = detect.depth * np.sin(detect.altitude)
        
        # Convert local (x, y, z) to world coordinates
        # world_location = sensor_transform.transform(carla.Location(x, y, z))
                   
        fw_vec = carla.Vector3D(x=detect.depth - 0.25) #adjust the distance for visibility
        carla.Transform(
            carla.Location(),
            carla.Rotation(
                pitch=current_rot.pitch + alt,
                yaw=current_rot.yaw + azi,
                roll=current_rot.roll)).transform(fw_vec)
                
        # Filtering out the points that are below the height threshold
        if fw_vec.z < height_threshold:
            continue
              
        def clamp(min_v, max_v, value):
            return max(min_v, min(value, max_v))

        norm_velocity = detect.velocity / velocity_range # range [-1, 1]
        r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
        g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
        b = int(abs(clamp(- 1.0, 0.0, -1.0 - norm_velocity)) * 255.0)
        # if abs(norm_velocity)<0.3:
        #     r, g, b = 0, 255, 0

        world.debug.draw_point(
            radar_data.transform.location + fw_vec,
            size=0.075,
            life_time=0.06,
            persistent_lines=False,
            color=carla.Color(r, g, b))
        
        radar_points.append([world_location.x, world_location.y, world_location.z])
        intensities.append(detect.intensity)        
    
    radar_points = np.array(radar_points)
    intensities = np.array(intensities)
    intensities = (intensities - intensities.min()) / (intensities.max() - intensities.min())

    return radar_points, intensities
        
# Ground Truth Data Collection
def get_ground_truth(world, prius):
    vehicles = world.get_actors().filter('vehicle.*')
    ground_truth = []
    for vehicle in vehicles:
        transform = vehicle.get_transform()
        bounding_box = vehicle.bounding_box
        ground_truth.append({
            'id': vehicle.id,
            'location': (transform.location.x, transform.location.y, transform.location.z),
            'dimensions': (bounding_box.extent.x, bounding_box.extent.y, bounding_box.extent.z),
            'distance': get_distance(prius, vehicle)
        })
    return ground_truth

# Store ground truth data
def save_ground_truth(world, frame_count, prius):
    gt_filename = os.path.join(PARENT_FOLDER, "ground_truth", f"ground_truth_{frame_count // 20:05d}.txt")
    ground_truth_data = get_ground_truth(world, prius)
    with open(gt_filename, 'w') as f:
        for gt in ground_truth_data:
            f.write(f"ID: {gt['id']}, Location: {gt['location']}, Dimensions: {gt['dimensions']}\n")

def get_distance(vehicle1, vehicle2):
    loc1 = vehicle1.get_location()
    loc2 = vehicle2.get_location()
    return math.sqrt((loc1.x - loc2.x) ** 2 + (loc1.y - loc2.y) ** 2 + (loc1.z - loc2.z) ** 2)

'''
# ==============================================================================
# -- Main loop-------- ---------------------------------------------------------
# ==============================================================================
'''

def main():
    camera_queue = queue.Queue()
    radar_queue = queue.Queue()

    crashed = False
    frame_count =0

    image = None
    writer = None

    # Initialize pygame
    pygame.init()

    # Set display parameters
    WIDTH, HEIGHT = 1280, 720
    DISPLAY = pygame.display.set_mode((WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
    DISPLAY.fill((0,0,0))
    pygame.display.set_caption("Dissertation CARLA Visualization")
    pygame.display.flip()

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        print("Successfully connected to CARLA client.")
        
        world = client.load_world('Town03')
        print('Loaded map')
   
        # Set up the simulator in synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds= 0.05 # fixed timestep (20 fps)
        #settings.no_rendering_mode = True
        world.apply_settings(settings)
        
        #Set up the TM in synchronous mode
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        # Set a seed so behaviour can be repeated if necessary
        #traffic_manager.set_random_device_seed(0)
        #random.seed(0)
      
        set_weather(world)

        global bp_lib 
        bp_lib = world.get_blueprint_library()

        ego_vehicle, ego_spawnpoint = spawn_ego_vehicle(world)
        camera, radar = attach_sensors(world, ego_vehicle)
        ego_vehicle.set_autopilot(True, 8000)

        spawn_npcs(world, ego_spawnpoint)

        Destination = ego_spawnpoint
        Destination.location.x +=100
        #Destination = world.get_map().get_spawn_points()[189]
        print("Destination: ", Destination.location)
    
        camera.listen(lambda data: camera_queue.put(data))   
        radar.listen(lambda data: radar_queue.put(data))
        #lidar.listen(lambda data: lidar_queue.put(data))

        # Get the world to camera matrix
        world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

        fov = 120.0

        # Calculate the camera projection matrix to project from 3D -> 2D
        K = build_projection_matrix(WIDTH, HEIGHT, fov)
        K_b = build_projection_matrix(WIDTH, HEIGHT, fov, is_behind_camera=True)
        edges = [[0,1], [1,3], [3,2], [2,0], [0,4], [4,5], [5,1], [5,7], [7,6], [6,4], [6,2], [7,3]]

        def point_in_canvas(pos, img_h, img_w):
            """Return true if point is in canvas"""
            if (pos[0] >= 0) and (pos[0] < img_w) and (pos[1] >= 0) and (pos[1] < img_h):
                return True
            return False

        traffic_manager.auto_lane_change(ego_vehicle, False)  # Disable lane changes
        traffic_manager.distance_to_leading_vehicle(ego_vehicle, 1.0)  # Maintain 5m distance
        
        while not crashed:
            world.tick()

            if not camera_queue.empty():
                image = camera_queue.get()
                camera_data = process_image(image)

                surface = pygame.surfarray.make_surface(camera_data.swapaxes(0, 1))
                DISPLAY.blit(surface, (0, 0))

            pygame.display.flip()
            
            if not radar_queue.empty():       
                raw_data = radar_queue.get()
            #radar_data = process_radar(raw_data)
                filtered_pcd, intensity= rad_callback(raw_data, world)
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    crashed = True
                elif event.type == pygame.KEYDOWN:
                    if event.key == K_ESCAPE or event.key == K_q:
                        crashed = True
                        raise KeyboardInterrupt
            
                    # Get the camera matrix 
            world_2_camera = np.array(camera.get_transform().get_inverse_matrix())
            
            if image is not None:
                frame_path = os.path.join(PARENT_FOLDER, "images", f"{image.frame:06d}")


                # Initialize the exporter
                writer = Writer(frame_path + '.png', WIDTH, HEIGHT)
            
            for npc in world.get_actors().filter('*vehicle*'):

                # Filter out the ego vehicle
                if npc.id != ego_vehicle.id:

                    bb = npc.bounding_box
                    dist = npc.get_transform().location.distance(ego_vehicle.get_transform().location)

                    # Filter for the vehicles within 50m
                    if dist < 50:

                    # Calculate the dot product between the forward vector
                    # of the vehicle and the vector between the vehicle
                    # and the other vehicle. We threshold this dot product
                    # to limit to drawing bounding boxes IN FRONT OF THE CAMERA
                        forward_vec = ego_vehicle.get_transform().get_forward_vector()
                        ray = npc.get_transform().location - ego_vehicle.get_transform().location
                        
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

                            # cv2.line(img, (int(x_min),int(y_min)), (int(x_max),int(y_min)), (0,0,255, 255), 1)
                            # cv2.line(img, (int(x_min),int(y_max)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
                            # cv2.line(img, (int(x_min),int(y_min)), (int(x_min),int(y_max)), (0,0,255, 255), 1)
                            # cv2.line(img, (int(x_max),int(y_min)), (int(x_max),int(y_max)), (0,0,255, 255), 1)
                            # Draw bounding box in pygame
                            rect = pygame.Rect(int(x_min), int(y_min), int(x_max - x_min), int(y_max - y_min))
                            pygame.draw.rect(DISPLAY, (255, 0, 0), rect, 2)
                            
                            # Add the object to the frame (ensure it is inside the image)
                            if (writer is not None) and x_min > 0 and x_max < WIDTH and y_min > 0 and y_max < HEIGHT: 
                                writer.addObject('vehicle', x_min, y_min, x_max, y_max)
                            
                                    
            '''
            # ==============================================================================
            # -- Data storage-------- ---------------------------------------------------------
            # ==============================================================================
            '''  
                
            if frame_count % 20 == 0 and writer is not None:
                # Save RGB image
                img_filename = os.path.join(PARENT_FOLDER, "images", f"{frame_count // 20:05d}.png")
                pygame.image.save(DISPLAY, img_filename)
                writer.save(frame_path + '.xml')



                if filtered_pcd is not None:
                    # radar_filename = os.path.join(PARENT_FOLDER, "pointcloud", f"pcd_{frame_count // 20:05d}.csv")
                    # np.savetxt(radar_filename, filtered_pcd, delimiter=',')

                    # Create point cloud object
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(filtered_pcd)  # XYZ points
                    pcd.colors = o3d.utility.Vector3dVector(np.tile(intensity[:, None], (1, 3)))  # Intensity mapped to RGB

                    ply_filename = os.path.join(PARENT_FOLDER, "pointcloud", f"pcd_{frame_count // 20:05d}.ply")
                    o3d.io.write_point_cloud(ply_filename, pcd)

                save_ground_truth(world, frame_count, ego_vehicle)
                
    
            location = ego_vehicle.get_location()
            distance = location.distance(Destination.location)
            if distance < 2.0:
                ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                print("Arrived destination.")
                #time.sleep(5)
                break
 
            time.sleep(0.01)
                    
            frame_count +=1
            pygame.display.flip()
            
    except KeyboardInterrupt:
        print('\nCancelling by user.')

        
    finally:
        camera.stop()
        radar.stop()
        #ego_vehicle.destroy()
        client.apply_batch([carla.command.DestroyActor(x) for x in world.get_actors()])
        print('Actors destroyed. \nQuitting...')

        
        # Revert synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        #settings.no_rendering_mode = False
        world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(False)

        pygame.quit()
        
        print("Bye!")


if __name__ == '__main__':
    main()
