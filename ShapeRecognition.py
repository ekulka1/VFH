##  https://www.youtube.com/watch?v=-OSVKbSsqT0&ab_channel=FlorentPoux

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


DATANAME_cube = "./Data/point_cloud_carton1.ply"
DATANAME_test = "./Data/test.ply"
print("Read Data")
#pcd = o3d.io.read_point_cloud('./Data/point_cloud_carton1.ply')
pcd = o3d.io.read_point_cloud('./Data/point_cloud_carton3.ply')
o3d.visualization.draw_geometries([pcd])

# Pre-data processing
pcd_center = pcd.get_center()
pcd.translate(-pcd_center)

#Statistical outlier filter (KNN)
nn = 16  #16
std_multiplier = 10  #10
filtered_pcd = pcd.remove_statistical_outlier(nn,std_multiplier)

outliners = pcd.select_by_index(filtered_pcd[1],invert=True)
outliners.paint_uniform_color([1,1,0])
filtered_pcd = filtered_pcd[0]
print("Outlier filter")
o3d.visualization.draw_geometries([filtered_pcd, outliners])

#Downsample
voxel_size = 0.000001
pcd_downsampled = filtered_pcd.voxel_down_sample(voxel_size = voxel_size)
#pcd_downsampled = pcd_downsampled.uniform_down_sample(every_k_points=5)

print(pcd_downsampled)
print("Downsample Vertex")
o3d.visualization.draw_geometries([pcd_downsampled])

#Estimating normals 
nn_distance = np.mean(pcd.compute_nearest_neighbor_distance())

radius_normals = nn_distance*4

pcd_downsampled.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normals, max_nn=16), fast_normal_computation=True)

pcd_downsampled.paint_uniform_color([0.6,0.6,0.6])
print("Estimating Normals")
o3d.visualization.draw_geometries([pcd_downsampled,outliners])

#Extracting and Setting Parameters

#front =  [ 0.46890696391959552, -0.091681421582104197, -0.8784763947451244 ]
#lookat = [
#				7.2192693111053309e-05,
#				-0.0002394739466446202,
#				-7.103437037577585e-05
#			]
#up = [ -0.1234325050737784, 0.97803625598854316, -0.16795683571419356 ]
#zoom = 0.20000000000000004


#pcd = pcd_downsampled
#o3d.visualization.draw_geometries([pcd]
#                                  #,zoom=zoom,front=front,lookat=lookat,up=up
#                                  )



#RANSAC PLANAR SEGMENTATION   TODO NOVA MAPA

#pt_to_plane_dist = 0.00002      # na zaklade mapy
#
#plane_model, iniliers = pcd.segment_plane(distance_threshold=pt_to_plane_dist,ransac_n=3,num_iterations=1000)
#[a,b,c,d] = plane_model
#print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f}x = 0")
#
#inilier_cloud = pcd.select_by_index(iniliers)
#outlier_cloud = pcd.select_by_index(iniliers, invert=True)
#inilier_cloud.paint_uniform_color([1.0,0,0])
#outlier_cloud.paint_uniform_color([0.6,0.6,0.6])
#
#print("RANSAC segmentation")
#
#o3d.visualization.draw_geometries([inilier_cloud, outlier_cloud]
#                                  #,zoom=zoom,front=front,lookat=lookat,up=up
#                                  )



#Multi-order RANSAC
max_plane_idx = 6
pt_to_plane_dist = 0.00002  # na zaklade pt_to_plane_dist

segment_models = {}
segments = {}
rest = pcd 

for i in range(max_plane_idx):
    colors = plt.get_cmap("tab20")(i)
    segment_models[i], iniliers = rest.segment_plane(distance_threshold=pt_to_plane_dist,ransac_n=3,num_iterations=1000)
    segments[i]=rest.select_by_index(iniliers)
    segments[i].paint_uniform_color(list(colors[:3]))
    rest = rest.select_by_index(iniliers,invert=True)
    print("pass",i,"/",max_plane_idx,"done")
    
print("Multi-order RANSAC")

o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)]+[rest]
                                  #,zoom=zoom,front=front,lookat=lookat,up=up
                                  )
                                  
                                  

#DBSCAN sur test

#labels = np.array(rest.cluster_dbscan(eps=0.05,min_points=5))
#
#max_label = labels.max()
##print(f"point cloud has {max_label + 1} clusters")
##
##colors = plt.get_cmap("tab10")(labels / (max_label if max_label > 0 else 1))
##colors[labels < 0] = 0
##rest.colors = o3d.utility.Vector3dVector(colors[:, :3])#
#print("DBSCAN sur test")
#o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)]+[rest]#,zoom=zoom,front=front,lookat=lookat,up=up
                                  #)
                                  
