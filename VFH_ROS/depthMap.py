#! /usr/bin/python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError

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


bridge = CvBridge()

class ProcessPoint:
    
    def __init__(self):
        rospy.init_node('process_point')
        #Nacitanie obr z kamery
        self.pictures = []
       
        self.rgb_left = rospy.Subscriber("/kinect2/qhd/image_color", sensorImage, self.get_left_Object)
        print("nacitanie left")
        rospy.sleep(5)
        self.rgb_right = rospy.Subscriber("/kinect2/qhd/image_color", sensorImage, self.get_right_Object)
        print("nacitanie right")
        #self.depth_sub = rospy.Subscriber("/kinect2/sd/image_ir_rect", Image, self.process_depth_cloud)
        
        self.cv_color_image = None
        self.cv_depth_image = None
        rospy.spin()

    
    def get_left_Object(self,image_msg):
        
        try:
            image_msg = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

            HSV = cv2.cvtColor(image_msg,cv2.COLOR_BGR2HSV)
            lower=np.array([-10, 100, 100])
            upper=np.array([16, 255, 255])
            Red_mask = cv2.inRange(HSV,lower, upper)
            mask = cv2.bitwise_and(image_msg, image_msg, mask = Red_mask)
            
            #cv2.imshow("Object mask", mask)
            #cv2.waitKey(0)  # Wait indefinitely until a key is pressed
            #cv2.destroyAllWindows()

            cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/DepthMaps/left_DM.png", mask)
            self.rgb_left.unregister()
            self.pictures.append(mask)
            self.process_cloud(self.pictures, False)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
        
    def get_right_Object(self,image_msg):
        
        try:
            image_msg = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

            HSV = cv2.cvtColor(image_msg,cv2.COLOR_BGR2HSV)
            lower=np.array([-10, 100, 100])
            upper=np.array([16, 255, 255])
            Red_mask = cv2.inRange(HSV,lower, upper)
            mask = cv2.bitwise_and(image_msg, image_msg, mask = Red_mask)
            
            #cv2.imshow("Object mask", mask)
            #cv2.waitKey(0)  # Wait indefinitely until a key is pressed
            #cv2.destroyAllWindows()

            cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/DepthMaps/right_DM.png", mask)
            self.rgb_right.unregister()
            self.pictures.append(mask)
            self.process_cloud(self.pictures, True)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
    
    
    ##                                              Processing  


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

    def process_cloud(self,cloud, is_color=True):
        print(len(cloud))
        if is_color == True:
            print("right image")
            #color_image = cloud
        else:
            print("left image")
            #depth_image = cloud
        if len(cloud) == 2:
            print("mam obe")

            left_img = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/DepthMaps/left_DM.png", cv2.IMREAD_GRAYSCALE)
            right_img = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/DepthMaps/right_DM.png", cv2.IMREAD_GRAYSCALE)
            depth_img = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/DepthMaps/right_DM.png", cv2.IMREAD_GRAYSCALE) 
            color_img = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/DepthMaps/right_DM.png")
            
            left_img = self.clipped_zoom(left_img,1.5)
            right_img = self.clipped_zoom(right_img,1.5)
            color_img = self.clipped_zoom(color_img,1.5)
            depth_img = self.clipped_zoom(depth_img,1.5)


            cv2.imshow("LIMG", left_img)
            cv2.waitKey(0)  
            cv2.destroyAllWindows()
            cv2.imshow("RIMG", right_img)
            cv2.waitKey(0)  
            cv2.destroyAllWindows()

            cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/DepthMaps/right_DM_zoomed.png", right_img)
            cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/DepthMaps/left_DM_zoomed.png", left_img)
            cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/DepthMaps/color_DM_zoomed.png", color_img)
            cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/DepthMaps/depth_DM_zoomed.png", depth_img)

            window_size = 7
            min_disp = 16
            nDispFactor = 14 # adjust this (14 is good)
            num_disp = 16*nDispFactor-min_disp
            stereo = cv2.StereoSGBM_create(minDisparity=min_disp,
                                    numDisparities=num_disp,
                                    blockSize=window_size,
                                    P1=8*3*window_size**2,
                                    P2=32*3*window_size**2,
                                    disp12MaxDiff=100,
                                    uniquenessRatio=15,
                                    speckleWindowSize=0,
                                    speckleRange=2,
                                    preFilterCap=63)

            disparity = stereo.compute(left_img,right_img)
            
            cv2.imshow("DIMG", disparity)
            cv2.waitKey(0)  
            cv2.destroyAllWindows()

            #cv2.imshow("color", color_img)
            #cv2.waitKey(0)  
            #cv2.destroyAllWindows()
            #cv2.imshow("depth", depth_img)
            #cv2.waitKey(0)  
            #cv2.destroyAllWindows()


if __name__ == '__main__':
    pp = ProcessPoint()