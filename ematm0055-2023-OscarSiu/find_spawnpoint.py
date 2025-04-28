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
import csv

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

world = client.load_world('Town03')
#world = client.get_world()

# Get spawn points in map
spawn_points = world.get_map().get_spawn_points()

print("Total number of spawn points: ", len(spawn_points))
spawn_index = 20
location = spawn_points[spawn_index].location

csv_filename = "carla_spawnpoints.csv"

with open(csv_filename, 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Index", "X", "Y","Z", "Pitch", "Yaw", "Roll"]) #Header Row

    for i, sp in enumerate(spawn_points):
        writer.writerow([i, sp.location.x, sp.location.y, sp.location.z, sp.rotation.pitch, sp.rotation.yaw, sp.rotation.roll])
        #print(f"Spawn Point {i}: x={sp.location.x}, y={sp.location.y}, z= {sp.location.z}")

print(f"Spawn points saved to {csv_filename}.")