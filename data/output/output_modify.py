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
    #print(f'transformation matrix: {trans_matrix}')

    Rotate = Rotation.from_matrix(trans_matrix)
    euler_angle = Rotate.as_euler('ZYZ', degrees=True)
    print(f'euler angle: {euler_angle}')
    
    if object_idx == 0:
        Rotate_for0 = Rotation.from_euler('YX', [90, 180], degrees=True)
        Rotate_new =  Rotate * Rotate_for0
        trans_matrix_new = Rotate_new.as_matrix()

        euler_angle_new = Rotate_new.as_euler('ZYZ', degrees=True)
        print(f'new euler angle for object_0 : {euler_angle_new}')
    elif object_idx == 1:
        Rotate_for1 = Rotation.from_euler('YZ', [-90, -90], degrees=True)
        Rotate_new =  Rotate * Rotate_for1
        trans_matrix_new = Rotate_new.as_matrix()

        euler_angle_new = Rotate_new.as_euler('ZYZ', degrees=True)
        print(f'new euler angle for object_1 : {euler_angle_new}')
    elif object_idx == 2:
        Rotate_for2 = Rotation.from_euler('YZ', [90, 180], degrees=True)
        Rotate_new =  Rotate * Rotate_for2
        trans_matrix_new = Rotate_new.as_matrix()

        euler_angle_new = Rotate_new.as_euler('ZYZ', degrees=True)
        print(f'new euler angle for object_2 : {euler_angle_new}')
    elif object_idx == 3:
        Rotate_for3 = Rotation.from_euler('XZ', [90, 180], degrees=True)
        Rotate_new =  Rotate * Rotate_for3
        trans_matrix_new = Rotate_new.as_matrix()

        euler_angle_new = Rotate_new.as_euler('ZYZ', degrees=True)
        print(f'new euler angle for object_3 : {euler_angle_new}')
    elif object_idx == 4:
        #Rotate_for4 = Rotation.from_euler('X', [0], degrees=True)
        #Rotate_new =  Rotate * Rotate_for4
        trans_matrix_new = trans_matrix

        euler_angle_new = euler_angle
        print(f'new euler angle for object_4 : {euler_angle_new}')
    elif object_idx == 5:
        Rotate_for5 = Rotation.from_euler('XZ', [90, 90], degrees=True)
        Rotate_new =  Rotate * Rotate_for5
        trans_matrix_new = Rotate_new.as_matrix()

        euler_angle_new = Rotate_new.as_euler('ZYZ', degrees=True)
        print(f'new euler angle for object_5 : {euler_angle_new}')


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
    mesh.scale(0.1, center=mesh.get_center())
    mesh_r = copy.deepcopy(mesh).rotate(trans_matrix_new)
    #mesh_r.scale(0.1, center=mesh_r.get_center())
    mesh_mv = copy.deepcopy(mesh_r).translate(center_point, relative=False)

    #print(f'mesh_mv center: {mesh_mv.get_center()}')


    print("visualizing...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(object)
    #o3d.visualization.draw_geometries([pcd, line_set, mesh_mv])

    #np.save(f"/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/object{object_idx}_center_point.npy", center_point)
    #np.save(f"/home/zhanfeng/camera_ws/src/ApproxMVBB/data/output/object{object_idx}_transform_matrix.npy", trans_matrix)
    
    lines = [f'Euler angle for object_{object_idx}:    ', f'{euler_angle_new}','\n', f'Center point for object_{object_idx}:    ', f'{center_point}', '\n\n\n']
    if object_idx == 0:
        with open('MVBB_output.txt', 'w') as f:
            f.writelines(lines)
    else:
        with open('MVBB_output.txt', 'a') as f:
            f.writelines(lines)
        

    return mesh, pcd, line_set, mesh_mv
    

#num_of_objects = 6
#for i in range(num_of_objects):
mesh0, pcd0, line_set0, mesh_mv0 = MVBB_results(0)
mesh1, pcd1, line_set1, mesh_mv1 = MVBB_results(1)
mesh2, pcd2, line_set2, mesh_mv2 = MVBB_results(2)
mesh3, pcd3, line_set3, mesh_mv3 = MVBB_results(3)
mesh4, pcd4, line_set4, mesh_mv4 = MVBB_results(4)
mesh5, pcd5, line_set5, mesh_mv5 = MVBB_results(5)

o3d.visualization.draw_geometries([mesh0, pcd0, line_set0, mesh_mv0, pcd1, line_set1, mesh_mv1, pcd2, line_set2, mesh_mv2, pcd3, line_set3, mesh_mv3, pcd4, line_set4, mesh_mv4, pcd5, line_set5, mesh_mv5])

