#! /usr/bin/python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from pyntcloud import PyntCloud 
from mpl_toolkits.mplot3d import Axes3D 
import pandas as pd
import open3d as o3d
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import cv2
import numpy as np
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image as pilImage
from sensor_msgs.msg import Image as sensorImage
from scipy.ndimage import zoom

#pcdload = o3d.io.read_point_cloud('./Data/point_cloud_new.ply')
#o3d.visualization.draw_geometries([pcdload])

bridge = CvBridge()

class ProcessPoint:
    def __init__(self):
        rospy.init_node('process_point')
        #Nacitanie obr z kamery
        print("nacitanie dat z kamery")
        #self.rgb_sub = rospy.Subscriber("/kinect2/qhd/image_color", sensorImage, self.process_image_cloud)
        #self.depth_sub = rospy.Subscriber("/kinect2/qhd/image_depth_rect", sensorImage, self.process_depth_cloud)
        
        self.rgb_object = rospy.Subscriber("/kinect2/qhd/image_color", sensorImage, self.getObject_color)
        #self.depth_object = rospy.Subscriber("/kinect2/qhd/image_depth_rect", sensorImage, self.getObject_depth)
        
        #self.depth_sub = rospy.Subscriber("/kinect2/sd/image_ir_rect", Image, self.process_depth_cloud)
        self.pictures = []
        self.mask = None
        self.contour_mask = None
        self.circle_object = None
        self.cv_color_image = None
        self.cv_depth_image = None
        self.object_points = []

        rospy.spin()

    def clipped_zoom(self,img, zoom_factor, **kwargs):

        h, w = img.shape[:2]

        zoom_tuple = (zoom_factor,) * 2 + (1,) * (img.ndim - 2)

        zh = int(np.round(h / zoom_factor))
        zw = int(np.round(w / zoom_factor))
        top = (h - zh) // 2
        left = (w - zw) // 2

        out = zoom(img[top:top+zh, left:left+zw], zoom_tuple, **kwargs)

        trim_top = ((out.shape[0] - h) // 2)
        trim_left = ((out.shape[1] - w) // 2)
        out = out[trim_top:trim_top+h, trim_left:trim_left+w]

        return out

    def getObject_color(self,image_msg):
        
        try:
            image_msg = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
    
            

            ###https://github.com/Amirag96/Red-color-detection/blob/master/red.py
            HSV = cv2.cvtColor(image_msg,cv2.COLOR_BGR2HSV)
            lower=np.array([-10, 100, 100])
            upper=np.array([16, 255, 255])
            Red_mask = cv2.inRange(HSV,lower, upper)
            self.mask = cv2.bitwise_and(image_msg, image_msg, mask = Red_mask)
            self.mask = self.clipped_zoom(self.mask,1.5)

            # Find contours in the mask
            mask_gray = cv2.cvtColor(self.mask, cv2.COLOR_BGR2GRAY)
            # Apply Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(mask_gray, (5, 5), 0)

            _, thresh = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


            if contours:
                self.contour_mask = self.mask
                largest_contour = max(contours, key=cv2.contourArea)
    
                # Get center
                """
                M = cv2.moments(largest_contour)
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                
                cv2.circle(self.contour_mask, (centroid_x, centroid_y), 5, (255, 255, 0), -1)
                """
                # Points around object

                for point in largest_contour:
                    x, y = point[0]
                    self.object_points.append((x, y))
                    #cv2.circle(self.contour_mask, (x, y), 1, (0, 255, 0), -1)
                

                 # Get the point farthest to the right and up
                """
                max_right_up_point = None
                max_x = -1
                min_y = float('inf')
                for point in largest_contour:
                    x, y = point[0]
                    if x > max_x or (x == max_x and y < min_y):
                        max_x = x
                        min_y = y
                        max_right_up_point = (x, y)

                if max_right_up_point is not None:
                    # Draw a circle at the farthest point
                    cv2.circle(self.contour_mask, max_right_up_point, 5, (0, 255, 255), -1)
                """
                #print(self.object_points)
                #cv2.imshow('Detected Object', self.mask)
                #cv2.waitKey(0)
                #cv2.destroyAllWindows()
                
                self.depth_object = rospy.Subscriber("/kinect2/qhd/image_depth_rect", sensorImage, self.getObject_depth)

                self.rgb_object.unregister()
                self.pictures.append(self.rgb_object)
                self.process_cloud(self.pictures, False)

                cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/color_result.png", self.mask)
            else:
                print("No red object detected.")

            
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
    
    def getObject_depth(self,image_msg):
        
      
        try:
            # Convert ROS Image message to OpenCV image
            self.cv_depth_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="16UC1") #16UC1
            print("saving depth image")
            self.cv_depth_image = self.cv_depth_image.astype(np.float32) / 25

            cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/testing_depth2.png", self.cv_depth_image)

            
            
            depth_read = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/testing_depth2.png")
            
            #cv2.imshow("Depth Rimport osead",depth_read)
            #cv2.waitKey(0)  
            #cv2.destroyAllWindows()

            depth_read = self.clipped_zoom(depth_read,1.5)
            #print("object points:" , self.object_points)
            for point in self.object_points:
                    #print("som tu")
                    x, y = point
                    #cv2.circle(depth_read, (x, y), 1, (0, 255, 0), -1)

            
            
            #cv2.imshow("Depth Read",depth_read)
            #cv2.waitKey(0)  
            #cv2.destroyAllWindows()


            contour = np.array(self.object_points)
            mask = np.zeros_like(depth_read, dtype=np.uint8)
            cv2.drawContours(mask, [contour], -1, (255, 255, 255), thickness=cv2.FILLED)
            result = cv2.bitwise_and(depth_read, mask)
            
            #cv2.imshow("Result", result)
            #cv2.waitKey(0)
            #cv2.destroyAllWindows()
            
            cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/depth_result.png", result)
            
            self.depth_object.unregister()
            self.pictures.append(self.cv_depth_image)
            self.process_cloud(self.pictures, False)
            #cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/depth_result.png", result)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
    

    """
    def process_image_cloud(self, image_msg):
        print("process color")
        try:
            
            self.cv_color_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough") #rgba8
            print("saving color image")
            
            cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/testing_color2.png", self.cv_color_image)    
            
            self.rgb_sub.unregister()
            self.pictures.append(self.cv_color_image)
            self.process_cloud(self.pictures, True)
            
            
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)

    def process_depth_cloud(self, image_msg):
        print("process depth")

        try:
            # Convert ROS Image message to OpenCV image
            self.cv_depth_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="16UC1") #16UC1
            print("saving depth image")
            self.cv_depth_image = self.cv_depth_image.astype(np.float32) / 25

            cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/testing_depth2.png", self.cv_depth_image)

            self.depth_sub.unregister()
            self.pictures.append(self.cv_depth_image)
            self.process_cloud(self.pictures, False)
            
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
    
    """
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
            depth_obj = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/depth_result.png")
            color_obj = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/color_result.png")
            
            cv2.imshow("Depth Read",depth_obj)
            cv2.waitKey(0)  
            cv2.destroyAllWindows()

            cv2.imshow("Color Read",color_obj)
            cv2.waitKey(0)  
            cv2.destroyAllWindows()

            color_obj = np.asarray(color_obj)
            depth_obj = np.asarray(depth_obj)
            color_obj = o3d.geometry.Image(color_obj)
            depth_obj = o3d.geometry.Image(depth_obj)

            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_obj,depth_obj)
            camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd,camera_intrinsic)
    
            pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

            
    
            #pcd_down = pcd.uniform_down_sample(10)    
            #o3d.visualization.draw_geometries([pcd_down])

            def RemoveNoiseStatistical(pc, nb_neighbors=10, std_ratio=2.0):  # 20 2.0
                cl, ind = pc.remove_statistical_outlier(
                nb_neighbors=nb_neighbors, std_ratio=std_ratio)

                return cl
            pcd = RemoveNoiseStatistical(pcd, nb_neighbors=50, std_ratio=0.5) #50 0.9
            
            o3d.io.write_point_cloud("/home/erik/catkin_ws/src/VFH/VFH_ROS/Data/testing_object_pc.ply", pcd)
            #o3d.visualization.draw_geometries([pcd])
            pcdload = o3d.io.read_point_cloud("/home/erik/catkin_ws/src/VFH/VFH_ROS/Data/testing_object_pc.ply")
            o3d.visualization.draw_geometries([pcdload])
            """depth_read = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/testing_depth2.png")
            color_raw = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/testing_color2.png")
            height, width, channels = depth_read.shape 
            #color_raw = cv2.resize(color_read, (512,424))
            depth_raw = depth_read
            cv2.imshow("Depth Read",depth_read)
            cv2.waitKey(0)  # Wait indefinitely until a key is pressed
            cv2.destroyAllWindows()
            cv2.imshow("Color Read",color_raw)
            cv2.waitKey(0)  # Wait indefinitely until a key is pressed
            cv2.destroyAllWindows()


            color_raw = np.asarray(color_raw)
            depth_raw = np.asarray(depth_raw)
            color_raw = o3d.geometry.Image(color_raw)
            depth_raw = o3d.geometry.Image(depth_raw)
            


            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw,depth_raw)
            camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd,camera_intrinsic)
    
            pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])

            o3d.io.write_point_cloud("/home/erik/catkin_ws/src/VFH/VFH_ROS/Data/testing_pc.ply", pcd)
    

            o3d.visualization.draw_geometries([pcd])"""

        


if __name__ == '__main__':
    pp = ProcessPoint()

    

    

