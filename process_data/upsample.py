import numpy as np
import open3d as o3d
import math

def read_test_ids():
    with open('./test_ids.txt', 'r') as f:
        ids = [line.strip() for line in f.readlines()]
    # ids=["00867262"]
    return ids

# normalize points within [-0.5, 0.5]
def normalize_model(points_with_normal, to_unit_sphere=False):
    assert(len(points_with_normal.shape) == 2 and points_with_normal.shape[1] == 3)
    points = points_with_normal[:,:3]
    #normalize to unit bounding box
    max_coord = points.max(axis=0)
    min_coord = points.min(axis=0)
    center = (max_coord + min_coord) / 2.0
    scale = (max_coord - min_coord).max()
    normalized_points = points - center
    if(to_unit_sphere):
        scale = math.sqrt(np.square(normalized_points).sum(-1).max())*2
    # normalized_points *= 0.95/scale
    normalized_points *= 1.0/scale
    return normalized_points

def sampling_data(id):
    import os
    obj_folder = "D:/CppProjects/ComplexGenTrim/top_10"
    files = os.listdir(obj_folder)
    obj_name = [f for f in files if f.__contains__(id) and f.endswith(".obj") and not f.__contains__("refine")][0]
    obj_file = os.path.join(obj_folder, obj_name)
    mesh = o3d.io.read_triangle_mesh(obj_file)
    mesh.compute_vertex_normals()
    pcl = mesh.sample_points_poisson_disk(number_of_points=100000, use_triangle_normal=True)

    # normalize pcl
    points = np.asarray(pcl.points)
    points = normalize_model(points)

    normals = np.asarray(pcl.normals)

    # TODO: sampling points uniformly on the edge of triangle

    pcl = o3d.geometry.PointCloud()
    pcl.points = o3d.utility.Vector3dVector(points)
    pcl.normals = o3d.utility.Vector3dVector(normals)

    o3d.io.write_point_cloud(f"D:/CppProjects/ComplexGenTrim/original_point_cloud/{id}_more_points.ply", pcl)

if __name__ == "__main__":
    # ids = ['00990000', '00990003', '00990023', '00990044']
    ids = read_test_ids()[:10]

    for id in ids:
        sampling_data(id)

    # o3d.visualization.draw_geometries([pcl])
