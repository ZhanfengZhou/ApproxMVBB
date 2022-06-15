import numpy as np
import open3d as o3d

for i in range(1,7):
    data = np.load(f"acc_{i}.npy")
    np.save(f"acc_{i}_x.npy", data[:, 0])
    np.save(f"acc_{i}_y.npy", data[:, 1])
    np.save(f"acc_{i}_z.npy", data[:, 2])
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data)
    o3d.visualization.draw_geometries([pcd])
