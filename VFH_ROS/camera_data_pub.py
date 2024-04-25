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
    def fill_holes(self,mask):

        kernel = np.ones((5,5), np.uint8)  # You can adjust the kernel size as needed
        closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return closing

    def getObject_color(self,image_msg):
        
        try:
            image_msg = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

            

            ###https://github.com/Amirag96/Red-color-detection/blob/master/red.py
            HSV = cv2.cvtColor(image_msg,cv2.COLOR_BGR2HSV)
            lower=np.array([-10, 100, 100])
            upper=np.array([16, 255, 255])
            Red_mask = cv2.inRange(HSV,lower, upper)

            Red_mask_closed = self.fill_holes(Red_mask)

            self.mask = cv2.bitwise_and(image_msg, image_msg, mask = Red_mask_closed)

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
    

                for point in largest_contour:
                    x, y = point[0]
                    self.object_points.append((x, y))
                    #cv2.circle(self.contour_mask, (x, y), 1, (0, 255, 0), -1)
                
                
                self.depth_object = rospy.Subscriber("/kinect2/qhd/image_depth_rect", sensorImage, self.getObject_depth)

                self.rgb_object.unregister()
                self.pictures.append(self.rgb_object)
                self.process_cloud(self.pictures, False)

                
                #self.mask = self.fill_holes(self.mask)
                #self.mask = cv2.bitwise_and(image_msg, image_msg, mask=Red_mask_closed)

                #cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/color_result_otherside.png", self.mask)
            else:
                print("No red object detected.")

            
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", e)
    
    def getObject_depth(self,image_msg):
        
        
        try:
            # Convert ROS Image message to OpenCV image
            self.cv_depth_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="16UC1") #16UC1
            print("saving depth image")


            self.cv_depth_image = self.cv_depth_image.astype(np.float32) / 4 #25
            
            
            

            #depth_array = np.array(self.cv_depth_image, dtype=np.float32)
            #depth_normalized = cv2.normalize(depth_array, None, alpha=100, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            #depth_equalized = cv2.equalizeHist(depth_normalized)
            #depth_colormap = cv2.applyColorMap(depth_equalized, cv2.COLORMAP_JET)

            #cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/testing_depth2.png", depth_colormap)
            #cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/testing_depth2.png", self.cv_depth_image)
            
            
            depth_read = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/testing_depth2.png")
            
            
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
            

            #cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/depth_result_otherside.png", result)
            
            self.depth_object.unregister()
            self.pictures.append(self.cv_depth_image)
            self.process_cloud(self.pictures, False)
            #cv2.imwrite("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/depth_result.png", result)

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
            #depth_obj = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/testing_depth4.png")
            
            color_obj = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/color_result.png")
            depth_obj = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/depth_result.png")

            #color_obj = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/color_result.png")
            
            
            #depth_obj = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/depth_result_otherside.png")
            #color_obj = cv2.imread("/home/erik/catkin_ws/src/VFH/VFH_ROS/pictures/color_result_otherside.png")

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
            
            print("pcd")
            o3d.visualization.draw_geometries([pcd])



            def RemoveNoiseStatistical(pc, nb_neighbors=20, std_ratio=0.02):  # 20 2.0
                cl, ind = pc.remove_statistical_outlier(
                nb_neighbors=nb_neighbors, std_ratio=std_ratio)

                return cl
            pcd = RemoveNoiseStatistical(pcd, nb_neighbors=100, std_ratio=0.2) #50 0.9

            print("remove noise statistical pcd")
            o3d.visualization.draw_geometries([pcd])
            
        
            #           point cloud editing
            


            def remove_clusters(pcd, eps, min_points):
                """
                Remove small clusters from a point cloud using DBSCAN clustering.

                Parameters:
                - pcd: The input point cloud as an open3d.geometry.PointCloud object.
                - eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other.
                - min_points: The number of samples in a neighborhood for a point to be considered as a core point.

                Returns:
                - A new open3d.geometry.PointCloud object with the largest cluster.
                """
                # DBSCAN clustering
                labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))

                # The largest cluster's label (excluding noise labeled as -1)
                max_label = labels.max()
                if max_label < 0:
                    print("No clusters found!")
                    return pcd

                # Identify the largest cluster
                largest_cluster_indices = np.where(labels == max_label)[0]
                # Select the points corresponding to the largest cluster
                largest_cluster = pcd.select_by_index(largest_cluster_indices)

                return largest_cluster

            main_pcd = remove_clusters(pcd,eps=0.00005,min_points=400)

            print("remove clusters")
            print(main_pcd.points)
            o3d.visualization.draw_geometries([main_pcd])


            #main_pcd = main_pcd.voxel_down_sample(voxel_size=0.000005)


            #           point cloud saving 
            o3d.io.write_point_cloud("/home/erik/catkin_ws/src/VFH/VFH_ROS/Data/testing_object_pc.ply", main_pcd)
            #o3d.visualization.draw_geometries([pcd])
            
            #pcdload = o3d.io.read_point_cloud("/home/erik/catkin_ws/src/VFH/VFH_ROS/Data/testing_object_pc.ply")
            pcdload = PyntCloud.from_file("/home/erik/catkin_ws/src/VFH/VFH_ROS/Data/testing_object_pc.ply")
            



             # convert Pyntcloud pcl to open3d pcl
            
            points = pcdload.points[['x', 'y', 'z']].values
            colors = pcdload.points[['red', 'green', 'blue']].values / 255.0  # Normalize colors to [0, 1]


            pcdload_o3d = o3d.geometry.PointCloud()

            pcdload_o3d.points = o3d.utility.Vector3dVector(points)
            pcdload_o3d.colors = o3d.utility.Vector3dVector(colors)

            print("pcdload_o3d")
            
            
            # define hyperparameters
            

            k_n = 400 # otherside 40  #400     #350-420 #50
            thresh = 0.095  # otherside 0.1 #0.095           #0.08

            pcd_np = np.zeros((len(pcdload.points),6))

            # find neighbors
            kdtree_id = pcdload.add_structure("kdtree")
            k_neighbors = pcdload.get_neighbors(k=k_n, kdtree=kdtree_id) 

            # calculate eigenvalues
            ev = pcdload.add_scalar_field("eigen_values", k_neighbors=k_neighbors)

            x = pcdload.points['x'].values 
            y = pcdload.points['y'].values 
            z = pcdload.points['z'].values 

            e1 = pcdload.points['e3('+str(k_n+1)+')'].values
            e2 = pcdload.points['e2('+str(k_n+1)+')'].values
            e3 = pcdload.points['e1('+str(k_n+1)+')'].values

            sum_eg = np.add(np.add(e1,e2),e3)
            sigma = np.divide(e1,sum_eg)
            sigma_value = sigma
            #pdb.set_trace()
            #img = ax.scatter(x, y, z, c=sigma, cmap='jet')

            # visualize the edges

            sigma = sigma>thresh

            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')

            img = ax.scatter(x, y, z, c=sigma, cmap='jet')
            #img = ax.scatter(x, y, z, c=sigma, cmap=plt.hot())

            fig.colorbar(img) 

            # Save the edges and point cloud
            thresh_min = sigma_value < thresh
            sigma_value[thresh_min] = 0
            thresh_max = sigma_value > thresh
            sigma_value[thresh_max] = 255

            pcd_np[:,0] = x
            pcd_np[:,1] = y
            pcd_np[:,2] = z
            pcd_np[:,3] = sigma_value

            edge_np = np.delete(pcd_np, np.where(pcd_np[:,3] == 0), axis=0) 

            clmns = ['x','y','z','red','green','blue']
            pcd_pd = pd.DataFrame(data=pcd_np,columns=clmns)
            pcd_pd['red'] = sigma_value.astype(np.uint8)

            #pcd_points = PyntCloud(pd.DataFrame(data=pcd_np,columns=clmns))
            pcd_points = PyntCloud(pcd_pd)
            edge_points = PyntCloud(pd.DataFrame(data=edge_np,columns=clmns))



            # convert Pyntcloud pcl to open3d pcl
            points = pcd_points.points[['x', 'y', 'z']].values
            colors = pcd_points.points[['red', 'green', 'blue']].values / 255.0  # Normalize colors to [0, 1]


            pcdpoints_o3d = o3d.geometry.PointCloud()

            pcdpoints_o3d.points = o3d.utility.Vector3dVector(points)
            pcdpoints_o3d.colors = o3d.utility.Vector3dVector(colors)

            print("point cloud with edges")        
            o3d.visualization.draw_geometries([pcdpoints_o3d])

            # convert Pyntcloud pcl to open3d pcl
            points = edge_points.points[['x', 'y', 'z']].values
            colors = edge_points.points[['red', 'green', 'blue']].values / 255.0  # Normalize colors to [0, 1]


            pcdedges_o3d = o3d.geometry.PointCloud()

            pcdedges_o3d.points = o3d.utility.Vector3dVector(points)
            pcdedges_o3d.colors = o3d.utility.Vector3dVector(colors)

            print("edges")
            o3d.visualization.draw_geometries([pcdedges_o3d])



            def mostly_right_and_above(point_cloud):
                rightmost_point = None
                lowest_point = None
                leftmost_point = None
                furthest_point = None


                for point in point_cloud:
                    x, y, z = point 

                    if rightmost_point is None or x > rightmost_point[0] and z > rightmost_point[2]:
                        rightmost_point = point
                    if lowest_point is None or y < lowest_point[1]:
                        lowest_point = point
                    if leftmost_point is None or x < leftmost_point[0]:
                        leftmost_point = point
                    if furthest_point is None or z < furthest_point[2]:
                        furthest_point = point
            
                return rightmost_point, lowest_point, furthest_point, leftmost_point
            
            def find_centroid(point_cloud):
                return np.mean(point_cloud, axis=0)

            def find_highest_center_point(point_cloud, centroid, range_threshold):
                highest_point = None
                max_height = -float('inf')

                for point in point_cloud:
                    if np.linalg.norm(point - centroid) < range_threshold:
                        if point[2] > max_height:
                            highest_point = point
                            max_height = point[2]

                return highest_point

            # Convert Open3D point cloud object to NumPy array
            pcdedges_np = np.asarray(pcdedges_o3d.points)
            rightmost_point, lowest_point, furthest_point, leftmost_point = mostly_right_and_above(pcdedges_np)


            pcd_np = np.asarray(pcdedges_o3d.points)
            centroid = find_centroid(pcd_np)
            range_threshold = 0.1  
            highest_center_point = find_highest_center_point(pcd_np, centroid, range_threshold)
            
            
            color_rightmost = [0, 0, 1]  # Blue color
            color_lowest = [0, 1, 0]  # Green color
            #color_furthest = [1, 0, 0]  # Red color
            color_leftmost = [1, 1, 0]  # Yellow color
            color_highest_center = [0, 1, 0]  
            #pcdedges_o3d.colors = o3d.utility.Vector3dVector([color_rightmost if (point == rightmost_point).all() else color_lowest if (point == lowest_point).all() else color_furthest if (point == furthest_point).all() else color_leftmost if (point == leftmost_point).all() else [0, 0, 0] for point in pcdedges_np])
            
            #if highest_center_point is not None:
            #    color_highest_center = [0, 1, 0]  
            #    pcdedges_o3d.colors = o3d.utility.Vector3dVector([color_highest_center if (point == highest_center_point).all() else [0, 0, 0] for point in pcd_np])
            
            pcdedges_o3d.colors = o3d.utility.Vector3dVector(
                [color_rightmost if (point == rightmost_point).all() else
                color_lowest if (point == lowest_point).all() else
                #color_furthest if (point == furthest_point).all() else
                color_leftmost if (point == leftmost_point).all() else
                color_highest_center if (point == highest_center_point).all() else
                [0, 0, 0] for point in pcd_np])
        
            print("extreme points")
            o3d.visualization.draw_geometries([pcdedges_o3d])

            # Create LineSet for connecting extreme points to the highest center point
            lines = [[rightmost_point, highest_center_point], [lowest_point, highest_center_point], [leftmost_point, highest_center_point]]
            line_colors = [[1, 0, 0] for _ in range(len(lines))]  # Red color for lines

            
            

            # parallel lines
            leftmost_to_highest_vector = highest_center_point - leftmost_point
            rightmost_to_highest_vector = -leftmost_to_highest_vector
            rightmost_to_highest_point = rightmost_point + rightmost_to_highest_vector
            
            new_line = [[rightmost_point, rightmost_to_highest_point]]
            
            end_to_furthest_line = [[rightmost_to_highest_point, leftmost_point]]

            lowest_to_highest_vector = highest_center_point - lowest_point
            rightmost_to_highest_vector = -lowest_to_highest_vector
            rightmost_to_highest_point_low = rightmost_point + rightmost_to_highest_vector
            right_line_front = [[rightmost_point, rightmost_to_highest_point_low]]

            centerpoint_to_rightmost_vector = highest_center_point - rightmost_point
            lowestpoint_to_rightmost_vector = -centerpoint_to_rightmost_vector
            lowest_point_to_rightmost_point_low = lowest_point + lowestpoint_to_rightmost_vector
            down_line_front = [[lowest_point, lowest_point_to_rightmost_point_low]]

            leftmost_to_lowest_vector = -lowest_to_highest_vector
            leftmost_to_lowest_point_back = leftmost_point + leftmost_to_lowest_vector
            left_line_to_back  = [[leftmost_point, leftmost_to_lowest_point_back]]

            rightmost_to_furthrest_vector_up = rightmost_point - rightmost_to_highest_point
            rightmost_to_furthrest_vector_down = - rightmost_to_furthrest_vector_up
            rightmost_down_to_furthrest_point = lowest_point_to_rightmost_point_low + rightmost_to_furthrest_vector_down
            right_line_to_back = [[lowest_point_to_rightmost_point_low, rightmost_down_to_furthrest_point]]

            
            right_line_back = [[rightmost_to_highest_point, rightmost_down_to_furthrest_point]]
            left_line_back = [[lowest_point, leftmost_to_lowest_point_back]]
            down_line_back = [[leftmost_to_lowest_point_back, rightmost_down_to_furthrest_point]]


            lines.extend(down_line_back)
            lines.extend(left_line_back)
            lines.extend(left_line_to_back)
            lines.extend(right_line_back)
            lines.extend(right_line_to_back)
            lines.extend(right_line_front)
            lines.extend(new_line)
            lines.extend(end_to_furthest_line)
            lines.extend(down_line_front)

            line_colors = [[1, 0, 0] for _ in range(len(lines))] 
            
            line_set = o3d.geometry.LineSet()
            line_set.points = o3d.utility.Vector3dVector([point for line in lines for point in line])
            line_set.lines = o3d.utility.Vector2iVector([[i, i + 1] for i in range(0, len(lines) * 2, 2)])
            line_set.colors = o3d.utility.Vector3dVector(line_colors)
            
            
            print("line set on cube pcd")
            o3d.visualization.draw_geometries([pcdedges_o3d, line_set])

            
            
            
            print("line set on original pcd")
            o3d.visualization.draw_geometries([pcdload_o3d, line_set])
            print("end")
            
            #o3d.visualization.draw_geometries([pcdedges_o3d])

if __name__ == '__main__':
    pp = ProcessPoint()

    

    

