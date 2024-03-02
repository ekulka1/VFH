#                                                           Testy s point cloudom 
# 1. Vypocitanie point cloudu a ulozenie point cloudu
# 2. uniform downsampling // vertex nefungoval (blank screen, point clouds : 1)
# 3. Normal point cloud
# 4. Metody pre pracu s point cloudom : https://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html


import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import cv2

if __name__ == '__main__':
    #                                                               Nacitanie  obr
    print("Read Redwppd dataset")    
    depth_raw = o3d.io.read_image("./DepthImages/Depth_cube3.png")
    color_raw = o3d.io.read_image("./pictures/cube3.jpg")
    
    #                                                               Spravenie rgbd
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw,depth_raw)
    print(rgbd)
    #                                                               Vypis Grayscale a depth obrazku
    plt.subplot(1,2,1)
    plt.title('Grayscale image')
    plt.imshow(rgbd.color)
    plt.subplot(1,2,2)
    plt.title('Depth image')
    plt.imshow(rgbd.depth)
    plt.show()
    
    #                                                              Vypocitanie point cloudu
    camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd,camera_intrinsic)
    
    pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
    
    # Save point cloud to file
    o3d.io.write_point_cloud("./Data/point_cloud_cube3.ply", pcd)
    
    #                                                               Zobrazenie point cloudu
    o3d.visualization.draw_geometries([pcd])
    
    #                                                               Nacitanie point clodu z dat
    pcdload = o3d.io.read_point_cloud('./Data/point_cloud_carton1.ply')
    print(pcdload)
    print(np.asarray(pcdload.points))
    #                                                               Zobrazenie point cloudu z dat
    o3d.visualization.draw_geometries([pcdload])
    
    
    #                                                               Downsampling point cloud with voxel / uniform
    print("downsample")
    downpcd = pcdload.voxel_down_sample(voxel_size = 0.000001)           
    #downpcd = pcdload.uniform_down_sample(10)
    print(downpcd)
    print(np.asarray(downpcd.points))
    #                                                               Zobrazenie downsamplu
    o3d.visualization.draw_geometries([downpcd])  
    
    
    
   #print("Load a ply point cloud, print it, and render it")
   #pcd = o3d.io.read_point_cloud("../../test_data/ICP/cloud_bin_2.pcd")
   #o3d.visualization.draw_geometries([pcd],
   #                                zoom=0.3412,
   #                                front=[0.4257, -0.2125, -0.8795],
   #                                lookat=[2.6172, 2.0475, 1.532],
   #                                up=[-0.0694, -0.9768, 0.2024])

   #print("Downsample the point cloud with a voxel of 0.02")
   #voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)
   #o3d.visualization.draw_geometries([voxel_down_pcd],
   #                                zoom=0.3412,
   #                                front=[0.4257, -0.2125, -0.8795],
   #                                lookat=[2.6172, 2.0475, 1.532],
   #                                up=[-0.0694, -0.9768, 0.2024])
   
   
    #                                                                Normals of downsampled point cloudu
    #PRESS N TO SEE NORMALS USE - and + TO CONTROL LENGTH OF NORMAL
    downpcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=50))
    #                                                                Zobrazenie normaloveho point cloudu
    o3d.visualization.draw_geometries([downpcd],point_show_normal=True)
    
    #                                                                 Vypisanie normaloveho vektoru na nultej pozicii 
    print(downpcd.normals[0])
    
    
    
    #                                                               TODO planar patch detection
#    assert downpcd.has_normals()
#
#    # using all defaults
#    oboxes = downpcd.detect_planar_patches(
#        normal_variance_threshold_deg=60,
#        coplanarity_deg=75,
#        outlier_ratio=0.75,
#        min_plane_edge_length=0,
#        min_num_points=0,
#        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=30))
#
#    print("Detected {} patches".format(len(oboxes)))
#
#    geometries = []
#    for obox in oboxes:
#        print("Oriented Bounding Box:")
#        print(obox)
#
#        # Check if the bounding box has valid dimensions
#        if all(e > 0 for e in obox.extent):
#            mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox)
#            mesh.paint_uniform_color(obox.color)
#            geometries.append(mesh)
#            geometries.append(obox)
#        else:
#            print("Invalid bounding box dimensions. Skipping.")
#
#    geometries.append(downpcd)
#
#    o3d.visualization.draw_geometries(geometries)

print("Remove Noise statistical")

def RemoveNoiseStatistical(pc, nb_neighbors=20, std_ratio=2.0):
    cl, ind = pc.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    return cl

#points = o3d.io.read_point_cloud('Data/point_cloud_carton1.ply')
print("original load")
o3d.visualization.draw_geometries([pcdload])

pcdRemoved = RemoveNoiseStatistical(pcdload, nb_neighbors=50, std_ratio=0.9)
print("removed")
o3d.visualization.draw_geometries([pcdRemoved])