import numpy as np
import open3d as o3d

for i in range(6):
    data = np.load(f"/home/zhanfeng/camera_ws/src/ApproxMVBB/data/input/object_{i}.npy")
    #np.save(f"object_{i}_x.npy", data[:, 0])
    #np.save(f"object_{i}_y.npy", data[:, 1])
    #np.save(f"object_{i}_z.npy", data[:, 2])
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data)
    o3d.visualization.draw_geometries([pcd])
