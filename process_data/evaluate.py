import open3d as o3d 
import trimesh
from tqdm import tqdm
import numpy as np
import math
import os

root = "D:/CppProjects/ComplexGenTrim/adv_experiment"

def complexgen_normalize_model(points_with_normal, to_unit_sphere=False):
    assert (len(points_with_normal.shape) == 2 and points_with_normal.shape[1] == 3)
    points = points_with_normal[:, :3]
    # normalize to unit bounding box
    max_coord = points.max(axis=0)
    min_coord = points.min(axis=0)
    center = (max_coord + min_coord) / 2.0
    scale = (max_coord - min_coord).max()
    # normalized_points = points - center
    # normalized_points *= 1.0/scale
    return center, scale


def chamfer_distance(mesh1, mesh2, num_samples=10000):
    # sampled_mesh1 = mesh1.sample_points_uniformly(number_of_points=math.ceil(mesh1.get_surface_area() * num_samples))
    # sampled_mesh2 = mesh2.sample_points_uniformly(number_of_points=math.ceil(mesh2.get_surface_area() * num_samples))
    sampled_mesh1 = o3d.geometry.PointCloud()
    sampled_mesh1.points = o3d.utility.Vector3dVector(np.array(mesh1.vertices))
    sampled_mesh2 = o3d.geometry.PointCloud()
    sampled_mesh2.points = o3d.utility.Vector3dVector(np.array(mesh2.vertices))

    o3d.io.write_point_cloud(f"sampled_mesh1.ply", sampled_mesh1)
    o3d.io.write_point_cloud(f"sampled_mesh2.ply", sampled_mesh2)

    dist1 = sampled_mesh1.compute_point_cloud_distance(sampled_mesh2)
    dist2 = sampled_mesh2.compute_point_cloud_distance(sampled_mesh1)
    chamfer_distance = (np.mean(dist1) + np.mean(dist2)) / 2
    return chamfer_distance

if __name__ == "__main__":
    ids = [line.strip() for line in open(f"test_ids.txt", "r")]
    ids = sorted(ids)
    avg_without_trim_chamfer = 0
    avg_trim_chamfer = 0
    for id in ids:
        gt_file = os.path.join(root, [f for f in os.listdir(root) if f.__contains__(id) and f.endswith(".obj") and f.__contains__("trimesh")][0])
        gt_mesh = trimesh.load(gt_file, force='mesh')
        center, scale = complexgen_normalize_model(gt_mesh.vertices)
        o3d_gt_mesh = o3d.geometry.TriangleMesh()
        o3d_gt_mesh.vertices = o3d.utility.Vector3dVector((gt_mesh.vertices - center) / scale)
        o3d_gt_mesh.triangles = o3d.utility.Vector3iVector(gt_mesh.faces)
        o3d.io.write_triangle_mesh(f"{id}_gt.ply", o3d_gt_mesh)

        without_trim_mesh = trimesh.load(f"{root}/{id}_geom_refine.obj", force='mesh')
        o3d_without_trim_mesh = o3d.geometry.TriangleMesh()
        o3d_without_trim_mesh.vertices = o3d.utility.Vector3dVector(without_trim_mesh.vertices)
        o3d_without_trim_mesh.triangles = o3d.utility.Vector3iVector(without_trim_mesh.faces)
        
        trim_mesh = trimesh.load(f"{root}/{id}_complexgen_trim.ply", force='mesh')
        o3d_trim_mesh = o3d.geometry.TriangleMesh()
        o3d_trim_mesh.vertices = o3d.utility.Vector3dVector(trim_mesh.vertices)
        o3d_trim_mesh.triangles = o3d.utility.Vector3iVector(trim_mesh.faces)

        
        without_trim_chamfer = chamfer_distance(o3d_without_trim_mesh, o3d_gt_mesh)
        trim_chamfer = chamfer_distance(o3d_trim_mesh, o3d_gt_mesh)
        avg_without_trim_chamfer += without_trim_chamfer
        avg_trim_chamfer += trim_chamfer

    avg_without_trim_chamfer /= len(ids)
    avg_trim_chamfer /= len(ids)
    print(f"avg_without_trim_chamfer: {avg_without_trim_chamfer}")
    print(f"avg_trim_chamfer: {avg_trim_chamfer}")
