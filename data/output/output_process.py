import numpy as np
import open3d as o3d
import copy
from scipy.spatial.transform import Rotation

def MVBB_results(object_idx):

    object = np.load(f'/home/zhanfeng/camera_ws/src/ApproxMVBB/data/input/object_{object_idx}.npy')
    
    max_point = np.load(f'/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/m_maxPoint{object_idx}.npy')
    min_point = np.load(f'/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/m_minPoint{object_idx}.npy')
    
    trans_matrix_x = np.load(f'/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/trans_matrix_{object_idx}x.npy')
    trans_matrix_y = np.load(f'/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/trans_matrix_{object_idx}y.npy')
    trans_matrix_z = np.load(f'/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/trans_matrix_{object_idx}z.npy')
    
    # MVBB
    trans_matrix = [
        trans_matrix_x,
        trans_matrix_y,
        trans_matrix_z
    ]
    print(f'transformation matrix: {trans_matrix}')

    Rotate = Rotation.from_matrix(trans_matrix)
    euler_angle = Rotate.as_euler('ZYZ', degrees=True)
    print(f'euler angle: {euler_angle}')
    
    up2 = max_point
    lp0 = min_point
    
    lp1 = [up2[0], lp0[1], lp0[2]]
    lp2 = [up2[0], up2[1], lp0[2]]
    lp3 = [lp0[0], up2[1], lp0[2]]
    up0 = [lp0[0], lp0[1], up2[2]]
    up1 = [up2[0], lp0[1], up2[2]]
    up3 = [lp0[0], up2[1], up2[2]]
    
    lp0 = np.matmul(trans_matrix, lp0)
    lp1 = np.matmul(trans_matrix, lp1)
    lp2 = np.matmul(trans_matrix, lp2)
    lp3 = np.matmul(trans_matrix, lp3)
    up0 = np.matmul(trans_matrix, up0)
    up1 = np.matmul(trans_matrix, up1)
    up2 = np.matmul(trans_matrix, up2)
    up3 = np.matmul(trans_matrix, up3)
    points = [lp0, lp1, lp3, lp2, up0, up1, up3, up2]
    #print(f'all points: {points}')

    center_point = np.matmul(trans_matrix, (max_point + min_point) / 2)
    print(f'center point: {center_point}')

    lines = [[0, 1], [0, 2], [1, 3], [2, 3],
             [4, 5], [4, 6], [5, 7], [6, 7],
             [0, 4], [1, 5], [2, 6], [3, 7]]
    colors = [[1, 0, 0] for i in range(len(lines))]
    line_set = o3d.cpu.pybind.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    
    
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    mesh_r = copy.deepcopy(mesh).rotate(trans_matrix)
    mesh_r.scale(0.04, center=mesh_r.get_center())
    mesh_mv = copy.deepcopy(mesh_r).translate(center_point, relative=False)

    print(f'mesh_mv center: {mesh_mv.get_center()}')


    print("visualizing...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(object)
    o3d.visualization.draw_geometries([pcd, line_set, mesh_mv])
    
    #np.save(f"/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/object{object_idx}_center_point.npy", center_point)
    #np.save(f"/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/object{object_idx}_transform_matrix.npy", trans_matrix)
    np.save(f"/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/object{object_idx}_euler_angle.npy", euler_angle)


num_of_objects = 6
for i in range(num_of_objects):
    MVBB_results(i) 