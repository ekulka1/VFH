#! /usr/bin/python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import open3d as o3d
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import cv2
import numpy as np
from cv_bridge import CvBridge
import os

#pcdload = o3d.io.read_point_cloud('./Data/point_cloud_new.ply')
#o3d.visualization.draw_geometries([pcdload])

bridge = CvBridge()

class ProcessPoint:
    def __init__(self):
        rospy.init_node('process_point')
        #Nacitanie obr z kamery
        print("nacitanie dat z kamery")
        self.rgb_sub = rospy.Subscriber("/kinect2/qhd/image_color_rect", Image, self.process_image_cloud)
        #self.depth_sub = rospy.Subscriber("/kinect2/sd/image_ir", Image, self.process_depth_cloud)
        self.depth_sub = rospy.Subscriber("/kinect2/sd/image_depth_rect", Image, self.process_depth_cloud)
        self.pictures = []
        self.cv_color_image = None
        self.cv_depth_image = None
        rospy.spin()

    def process_image_cloud(self, image_msg):
        print("process color")
        try:
            
            self.cv_color_image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
            print("saving image")
            cv2.imwrite("./pictures/testing_color.jpg", self.cv_color_image)    
            test = cv2.imread("./pictures/testing_color.jpg")
            #cv2.imshow("Saved image", test)
            #cv2.imshow("Saved Image", cv_image)
            #cv2.waitKey(0)  # Wait indefinitely until a key is pressed
            #cv2.destroyAllWindows()  # Close all OpenCV windows

            self.rgb_sub.unregister()
            self.pictures.append(self.cv_color_image)
            self.process_cloud(self.pictures, True)
            
            
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)

    def process_depth_cloud(self, image_msg):
        print("process depth")

        try:

            # Convert ROS Image message to numpy array
            #depth_image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
            
            depth_image = np.frombuffer(image_msg.data, dtype=np.uint16).reshape((image_msg.height, image_msg.width))
            depth_map = o3d.geometry.Image(depth_image)
            # Process depth image using Open3D
            #depth_map = o3d.geometry.Image(depth_image)
            #camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
            #pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_map, camera_intrinsic)
            #pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
            #o3d.io.write_point_cloud("./Data/point_cloud_test.ply", pcd)
            #o3d.visualization.draw_geometries([pcd])
            #self.cv_depth_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="16UC1")
            ##################################depth_image = depth_image.astype(np.float32) / 1000.0  # Convert to meters

             #Save the image using OpenCV
            #print("saving image")
            cv2.imwrite("./pictures/image_depth_test_new_new.jpg", depth_map)    
            #depth_read = cv2.imread("./pictures/image_depth_test_new_new.jpg")
            #self.depth_image = cv_image.copy()
            #self.process_images = True
            #cv2.imshow("Saved image", depth_read)

            #cv2.imshow("Saved Image", cv_image)
            cv2.waitKey(0)  # Wait indefinitely until a key is pressed
            cv2.destroyAllWindows()  # Close all OpenCV windows
            self.depth_sub.unregister()
            self.pictures.append(depth_map)
            self.process_cloud(self.pictures, False)
            
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
    



    def process_cloud(self,cloud, is_color=True):
        print(len(cloud))
        if is_color == True:
            print("Color image")
            #color_image = cloud
        else:
            print("depth image")
            #depth_image = cloud
        if len(cloud) == 2:
            print("mam obe")
        
        #color_image = cv2.imread("color_image.png")
        #depth_image = cv2.imread("depth_image.png", cv2.IMREAD_UNCHANGED)  # Convert depth to meters
        #depth_image = np.float32(depth_image)/1000
        #print("Color image shape:", color_image.shape)
        #print("Depth image shape:", depth_image.shape)

        
    
    # Create point cloud
        
    
    
        #o3d.visualization.draw_geometries([pcd])
        depth_image = o3d.io.read_image("./pictures/image_depth_test_new_new.jpg")
        color_image = o3d.io.read_image("./pictures/testing_color.jpg")

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image,depth_image)
        camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd,camera_intrinsic)
    
        pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
    
        o3d.io.write_point_cloud("./Data/point_cloud_test.ply", pcd)
    

        o3d.visualization.draw_geometries([pcd])
        #print(rgbd)"""
        
        """depth_raw = o3d.io.read_image("./pictures/image_depth_test.jpg")
            color_raw = o3d.io.read_image("./pictures/image_color_test.jpg")
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw,depth_raw)
            camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd,camera_intrinsic)
    
            pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
    
            #o3d.io.write_point_cloud("./Data/point_cloud_test.ply", pcd)
    

            #o3d.visualization.draw_geometries([pcd])
            #print(rgbd)"""



        """if hasattr(self, 'rgb_image') and hasattr(self, 'depth_image'):
            # Convert depth image to float32 for Open3D
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(self.rgb_image), o3d.geometry.Image(self.depth_image))

            # Camera intrinsic parameters for Kinect v2
            camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
            # Create point cloud
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, camera_intrinsic)

            # Transform point cloud (optional)
            pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

            o3d.io.write_point_cloud("./Data/point_cloud_test.ply", pcd)

            o3d.visualization.draw_geometries([pcd])"""

        


if __name__ == '__main__':
    pp = ProcessPoint()

    

    

