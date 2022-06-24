import numpy as np
import open3d as o3d
import copy
from scipy.spatial.transform import Rotation

def MVBB_results(object_idx):

    object = np.load(f'/home/zhanfeng/camera_ws/src/ApproxMVBB/data/input_seperate/image{object_idx}_cleaned.npy')
    

    print("visualizing...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(object)
    return pcd



pcd= MVBB_results(4)
o3d.visualization.draw_geometries([pcd])
