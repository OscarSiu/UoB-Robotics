import numpy as np
import pandas as pd
import open3d as o3d
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.cm as cm

# Load the PLY file
file_path = "sensor_data\straightline_rain_90\pointcloud\pcd_00004.ply"
pcd = o3d.io.read_point_cloud(file_path)
points = np.asarray(pcd.points)
colors = np.asarray(pcd.colors)
# print(intensity)

# Check the shape of the NumPy array
print("Shape of points array:", points.shape)
print("Num of intensity points:", colors.shape[0])

'""""""""""""""""""""""""""""""""Plot""""""""""""""""""""""""""""""""""""""""""""""'
intensity = np.mean(colors, axis=1)  # Average RGB for grayscale intensity
colormap = cm.viridis(intensity)[:, :3]  # Extract only RGB channels from colormap

pcd.colors = o3d.utility.Vector3dVector(colormap)
# o3d.visualization.draw_geometries([pcd], window_name="Point Cloud with Viridis Colormap")

centroid = np.mean(points, axis=0)
print("Centroid:", centroid)
bounding_box = np.ptp(points, axis=0)  # Range along each axis
print("Bounding Box Dimensions:", bounding_box)

# o3d.io.write_point_cloud("PointCloud\processed_point_cloud.ply", pcd)
# np.savetxt("PointCloud\processed_point_cloud.csv", points, delimiter=",", header="x,y,z")

'""""""""""""""""""""""""""""""""Add Noise""""""""""""""""""""""""""""""""""""""""""""""'
rain_intensity = 90  # Adjust for different conditions
# fog_intensity = 0.05  # Adjust for different conditions

# Add Gaussian noise to new points
max_noise = 0.02  # Maximum noise intensity
noise_std = (rain_intensity / 100) * max_noise  # Scale noise by rain intensity
noise = np.random.normal(0, noise_std, points.shape)  # Gaussian noise
noisy_points = points + noise  # Apply noise

pcd.points = o3d.utility.Vector3dVector(noisy_points)
print("Shape of noisy points array:", noisy_points.shape)

o3d.visualization.draw_geometries([pcd], window_name="Point Cloud with Gaussian Noise")

'""""""""""""""""""""""""""""""""DBSCAN Clustering""""""""""""""""""""""""""""""""""""""""""""""'
clustering = DBSCAN(eps=1, min_samples=10).fit(noisy_points) # eps = max dist between pts
labels = clustering.labels_

# Extract unique clusters
unique_labels = set(labels)
num_clusters = len(unique_labels) - (1 if -1 in labels else 0)  # Exclude noise

print(f"Number of clusters: {num_clusters}")
print(f"Total number of points: {list(labels).count(-1)}")

# Cluster information storage
cluster_info = []

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Assign colors to clusters
colors = [plt.cm.jet(each) for each in np.linspace(0, 1, len(unique_labels))]

for k, col in zip(unique_labels, colors):
    if k == -1:
        continue  # Skip noise
    
    class_member_mask = labels == k
    cluster_points = noisy_points[class_member_mask]
    
    # Compute centroid and bounding box
    centroid = np.mean(cluster_points, axis=0)
    bounding_box = np.ptp(cluster_points, axis=0)
    
    cluster_info.append((k, centroid, bounding_box))
    
    print(f"Cluster {k}: Centroid = {centroid}, Bounding Box Dimensions = {bounding_box}")
    
    ax.scatter(cluster_points[:, 0], cluster_points[:, 1], cluster_points[:, 2],
               c=[col], edgecolors='k', s=20, label=f"Cluster {k}")

#ax.invert_xaxis()  # Flip X if needed
# ax.invert_yaxis()  # Flip Y if needed
ax.view_init(elev=7, azim=-2)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("DBSCAN Clustering with Gaussian Noise")
plt.legend()
plt.show()


# # Matplotlib Visualization
# fig = plt.figure(figsize=(10, 7))
# ax = fig.add_subplot(111, projection='3d')

# sc = ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=colormap, s=1)
# plt.colorbar(cm.ScalarMappable(cmap="viridis"), label="Intensity (Viridis)")

# ax.set_xlabel("X Axis")
# ax.set_ylabel("Y Axis")
# ax.set_zlabel("Z Axis")
# ax.set_title("Point Cloud with Viridis Intensity Mapping")

# plt.show()
