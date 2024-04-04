#                                                                           Zbytocny

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import cv2

if __name__ == '__main__':
    #device = o3d.core.Device('CPU:0')
    #tum_data = o3d.data.SampleTUMRGBDImage()
    
    depth = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/testing_color2.png", 0)
    color = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/testing_color2.png", 1)  # Assuming color image, change to 0 if grayscale

    
            

    depth16 = depth.astype("uint16")
    color16 = color.astype("uint16")
    
    #color = np.asarray(color)
    #depth = np.asarray(depth)
    #color = o3d.geometry.Image(color)
    #depth = o3d.geometry.Image(depth)
    
    intrinsic = o3d.core.Tensor([[535.4, 0, 320.1], [0, 539.2, 247.6],
                                 [0, 0, 1]])
    
    # Use o3d.t.geometry.Image for color and depth
    #color_image = o3d.t.geometry.Image(color16)
    #depth_image = o3d.t.geometry.Image(depth16)
    color_image = o3d.t.geometry.Image(tensor=o3d.core.Tensor(color16))
    depth_image = o3d.t.geometry.Image(tensor=o3d.core.Tensor(depth16))


    rgbd = o3d.t.geometry.RGBDImage(color_image, depth_image)

    #pcd = o3d.t.geometry.PointCloud.create_from_rgbd_image(rgbd,
    #                                                       intrinsic,
    #                                                       depth_scale=7000.0,
    #                                                       depth_max=10.0)
    
    #o3d.visualization.draw([pcd])

    rgbd_reproj = o3d.cpu.pybind.t.geometry.PointCloud.project_to_rgbd_image(1000,
                                            1000,
                                            intrinsic,
                                            depth_scale=7000.0,
                                            depth_max=10.0)

    fig, axs = plt.subplots(1, 2)
    axs[0].imshow(np.asarray(rgbd_reproj.color.to_legacy()))  # Use color instead of color16
    axs[1].imshow(np.asarray(rgbd_reproj.depth.to_legacy()))
    
    # Save the depth values as a numpy array
    np.save("/home/erik/catkin_ws/src/VFH/VFH_ROS/Data/depth_values_testing_color2.npy", np.asarray(rgbd_reproj.depth.to_legacy()))
    
    img = cv2.applyColorMap(np.asarray(rgbd_reproj.depth.to_legacy(), dtype=np.uint8), cv2.COLORMAP_VIRIDIS)
    cv2.imshow("Depth Image",img)
    cv2.waitKey(0)
    
        # Save the depth image
    cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/Grayscaled_Image_testing_color2.png", np.asarray(rgbd_reproj.depth.to_legacy()))
        # Save the depth image
    cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/Depth_Image.png_testing_color2.png", img)
    
    plt.show()
