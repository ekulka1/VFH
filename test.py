import numpy as np
import open3d as o3d


pointEdges = o3d.io.read_point_cloud('Data/pointcloud_edges_carton1.ply')
edges = o3d.io.read_point_cloud('Data/edges_carton1.ply')

o3d.visualization.draw_geometries([pointEdges])
o3d.visualization.draw_geometries([edges])


#outlier remover

pcd = o3d.io.read_point_cloud('./Data/point_cloud_carton1.pcd')

voxel_size = 0.000001
pcd_downsampled = pcd.voxel_down_sample(voxel_size = voxel_size)


def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    o3d.visualization.draw_geometries([outlier_cloud])

print("Statistical oulier removal")
cl, ind = pcd_downsampled.remove_statistical_outlier(nb_neighbors=20,
                                                    std_ratio=2.0)
display_inlier_outlier(pcd_downsampled, ind)
