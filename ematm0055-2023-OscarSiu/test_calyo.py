#!/usr/bin/env python

# Author: Oscar Siu (oscarsiu21@gmail.com)
# Student number: 2441322
# Last Modified: 2025-02-13
# Version: 3.1
# This work is created for MSc Robotics Dissertation Project at 
# University of Bristol and University of West of England

# Description: Straight line scenario

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

if not os.path.exists("output"):
	os.mkdir("output")
    #os.makedirs("output/data", exist_ok=True)

# Callback functions for sensors
def process_image(image):
    array = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))
    array = array[:, :, :3][:, :, ::-1] # Keep RGB part
    return array

def process_radar(radar_data):
    radar_points = []
    for detection in radar_data:
        radar_points.append([detection.depth, detection.azimuth, detection.altitude, detection.velocity])
    radar_points = np.array(radar_points)
    return radar_points

# def lidar_callback(point_cloud, point_list):
#     data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.float32)).reshape(-1, 4)
#     intensity = data[:, -1] + 1e-8  
#     intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 50))
#     int_color = np.c_[
#         np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
#         np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
#         np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])
#     ]
#     points = data[:, :-1]
#     points[:, 0] = -points[:, 0]
#     point_list.points = o3d.utility.Vector3dVector(points)
#     point_list.colors = o3d.utility.Vector3dVector(int_color)
        
def main():

    rain_intensity = 70.0
    fog_density = 0.0
        
    image_queue = queue.Queue()
    radar_queue = queue.Queue()

    frame_count =0

    # Initialize pygame
    pygame.init()
    WIDTH, HEIGHT = 1280, 720
    display = pygame.display.set_mode((WIDTH, HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
    display.fill((0,0,0))
    pygame.display.set_caption("Scenario 1: Multiple Vehicles Straight line")
    pygame.display.flip()
    
    try:
        # Connect to the CARLA simulator
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        print("Successfully connected to CARLA client.")

        # Load the map
        world = client.load_world('Town03')
        print('Loaded map')
               
        # Set up the simulator in synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds= 0.05 # fixed timestep (20 fps)
        world.apply_settings(settings)
        
        #Set up the TM in synchronous mode
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)
        # Set a seed so behaviour can be repeated if necessary
        #traffic_manager.set_random_device_seed(0)
        #random.seed(0)
      
        weather = carla.WeatherParameters(
            sun_altitude_angle=45.0, # Day: 45.0, Night: -10, Sunset: 0.5
            sun_azimuth_angle=0.0,
            cloudiness=20.0,
            precipitation=rain_intensity, precipitation_deposits=0.0,  # Rain intensity
            wind_intensity=20.0,
            fog_density=fog_density, fog_distance=0.75, fog_falloff=0.1,
            wetness=0.0,
            scattering_intensity=0.0, mie_scattering_scale=0.03, rayleigh_scattering_scale=0.0331,
        )
        world.set_weather(weather)
        print('Set weather: ', weather)

        #spectator = world.get_spectator()

        # # Get spawn points in map
        # spawn_points = world.get_map().get_spawn_points()

        # for i, spawn_point in enumerate(spawn_points):
        #     # Draw in the spectator window the spawn point index
        #     world.debug.draw_string(spawn_point.location, str(i), life_time=10000)
        #     # Draw arrow to visualize orientation of the spawn pointW
        #     world.debug.draw_arrow(spawn_point.location, spawn_point.location + spawn_point.get_forward_vector(), life_time=10000)
        
        '''
        # ==============================================================================
        # -- Actors --------------------------------------------------------------------
        # ==============================================================================
        '''
        # Spawn ego vehicle
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find('vehicle.toyota.prius')
        vehicle_bp.set_attribute('role_name', 'hero')
        vehicle_bp.set_attribute('color', '85,229,167')
        vehicle_spawn_point = world.get_map().get_spawn_points()[20]
        ego_vehicle = world.spawn_actor(vehicle_bp, vehicle_spawn_point)
        
        # Get the bounding box of the vehicle
        bounding_box = ego_vehicle.bounding_box
        dimensions = bounding_box.extent  # Extent gives half-lengths in x, y, z

        # Calculate full dimensions
        length = 2 * dimensions.x
        width = 2 * dimensions.y
        height = 2 * dimensions.z

        print(f"Vehicle Dimensions: Length={length}m, Width={width}m, Height={height}m")
        
        ego_vehicle.set_autopilot(True, 8000)
        
        # Spawn NPC vehicles
        #npc_sp = [148, 106, 104, 20, 18]      
        npc_sp = [186, 181, 184, 19, 18]      

        # for i in range(len(npc_sp)):
        #     npc_bp = random.choice(blueprint_library.filter('vehicle.*'))
        #     npc_transform = world.get_map().get_spawn_points()[npc_sp[i]]
        #     vehicle = world.try_spawn_actor(npc_bp, npc_transform)
            
        #     if vehicle:
        #         actor_list.append(vehicle)
        #         vehicle.set_autopilot(True, 8000)
        
        npc_bp = random.choice(blueprint_library.filter('vehicle.*'))
        v1 = world.spawn_actor(npc_bp, carla.Transform(carla.Location(
            vehicle_spawn_point.location.x+50, vehicle_spawn_point.location.y-3, 
            vehicle_spawn_point.location.z)))

        v2 = world.spawn_actor(npc_bp, carla.Transform(carla.Location(
            vehicle_spawn_point.location.x+110, vehicle_spawn_point.location.y, 
            vehicle_spawn_point.location.z)))
        
        v3 = world.spawn_actor(npc_bp, carla.Transform(carla.Location(
            vehicle_spawn_point.location.x+30, vehicle_spawn_point.location.y+3, 
            vehicle_spawn_point.location.z)))
                
        print(f'Spawned {len(npc_sp)} vehicles')

        Destination = vehicle_spawn_point
        Destination.location.x +=100
        #Destination = world.get_map().get_spawn_points()[189]
        print("Destination: ", Destination.location)
         
        # RGB Camera setup
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', f'{WIDTH}')
        camera_bp.set_attribute('image_size_y', f'{HEIGHT}')
        camera_bp.set_attribute('fov', '90')

        camera_spawn_point = carla.Transform(carla.Location(x=-5, z=2.4))  # on top of the cybertruck
        camera_sensor = world.spawn_actor(camera_bp, camera_spawn_point, attach_to=ego_vehicle)
        camera_sensor.listen(lambda data: image_queue.put(data))

        # # LIDAR setup      
        # lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        # lidar_bp.set_attribute('channels',str(32))
        # lidar_bp.set_attribute('points_per_second',str(90000))
        # lidar_bp.set_attribute('rotation_frequency',str(40))
        # lidar_bp.set_attribute('range','50')
      
        # lidar_transform = carla.Transform(carla.Location(0,0,2.4),carla.Rotation(0,0,0))
        # lidar_sen = world.spawn_actor(lidar_bp,lidar_transform,attach_to=ego_vehicle)
        # #lidar_sen.listen(lambda point_cloud: point_cloud.save_to_disk('tutorial/new_lidar_output/%.6d.ply' % point_cloud.frame))

        # Calyo Sensor setup
        radar_bp = blueprint_library.find('sensor.other.calyopulse')
        radar_bp.set_attribute('horizontal_fov', '70')
        radar_bp.set_attribute('vertical_fov', '45')
        radar_bp.set_attribute('range', '30')
        radar_bp.set_attribute('points_per_second', '50000')

        radar_spawn_point = carla.Transform(carla.Location(x=2.0, z=1.0))  # front bumper
        radar_sensor = world.spawn_actor(radar_bp, radar_spawn_point, attach_to=ego_vehicle)

        # Visualization functions
        def rad_callback(radar_data):
            radar_points = []

            velocity_range = 7.5 # m/s
            current_rot = radar_data.transform.rotation
            height_threshold = -0.1
            
            for detect in radar_data:
                azi = math.degrees(detect.azimuth)
                alt = math.degrees(detect.altitude)
                x = detect.depth * np.cos(detect.azimuth) * np.cos(detect.altitude)
                y = detect.depth * np.sin(detect.azimuth) * np.cos(detect.altitude)
                z = detect.depth * np.sin(detect.altitude)
                
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
                b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
                world.debug.draw_point(
                    radar_data.transform.location + fw_vec,
                    size=0.075,
                    life_time=0.06,
                    persistent_lines=False,
                    color=carla.Color(r, g, b))
                
                radar_points.append([detect.depth, detect.azimuth, detect.altitude, detect.velocity, x, y, z])
            radar_points = np.array(radar_points)
            return radar_points

        radar_sensor.listen(lambda data: radar_queue.put(data))
        
        traffic_manager.auto_lane_change(ego_vehicle, False)  # Disable lane changes
        traffic_manager.distance_to_leading_vehicle(ego_vehicle, 1.0)  # Maintain 5m distance
        
        while True:
            world.tick()
            time.sleep(0.01)

            image = image_queue.get()
            array = process_image(image)
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))
            pygame.display.flip()
            
            # Event handling for pygame window
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
            
            if not radar_queue.empty():       
                raw_data = radar_queue.get()
            filtered_pcd = rad_callback(raw_data)

#            Save RGB image
            img_filename = f'output/frame_{frame_count:05d}.png'
            pygame.image.save(display, img_filename)


            if filtered_pcd is not None:
                radar_filename = f'output/radar_{frame_count:05d}.csv'
                np.savetxt(radar_filename, filtered_pcd, delimiter=',')     
            
            # Apply Control
            # for actor in actor_list:
            #     actor.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))
            #ego_vehicle.apply_control(carla.VehicleControl(throttle=0.8, steer=0.0))
            location = ego_vehicle.get_location()
            distance = location.distance(Destination.location)
            if distance < 2.0:
                ego_vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
                print("Arrived destination.")
                #time.sleep(5)
                break
            

            frame_count +=1
            # spectator_transform = carla.Transform(
            #     ego_vehicle.get_transform().location + carla.Location(x=-10, z=5), 
            #     ego_vehicle.get_transform().rotation)
            # spectator.set_transform(spectator_transform)
            
            pygame.display.flip()
    
    except KeyboardInterrupt:
        print('\nCancelling by user.')
        
    finally:
        camera_sensor.destroy()
        radar_sensor.destroy()
        ego_vehicle.destroy()
        client.apply_batch([carla.command.DestroyActor(x) for x in world.get_actors()])
        
        # Revert synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
        traffic_manager.set_synchronous_mode(False)
        
        pygame.quit()

        print("Bye!")

        
if __name__ == '__main__':
    main()