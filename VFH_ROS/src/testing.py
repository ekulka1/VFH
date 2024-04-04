import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
import open3d as o3d

# Initialize Open3D visualization window
vis = o3d.visualization.Visualizer()
vis.create_window()

# Callback function to process color and depth data
def image_callback(color_data, depth_data):
    # Convert ROS Image messages to numpy arrays
    color_image = np.frombuffer(color_data.data, dtype=np.uint8).reshape((color_data.height, color_data.width, 3))
    depth_image = np.frombuffer(depth_data.data, dtype=np.uint16).reshape((depth_data.height, depth_data.width))

    # Convert depth image to float32 for Open3D
    depth_image = depth_image.astype(np.float32) / 1000.0  # Convert to meters

    # Create RGBD image from color and depth
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_image, depth_image)

    # Create point cloud from RGBD image
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)

    # Visualize point cloud
    vis.add_geometry(pcd)
    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()

def main():
    global intrinsic

    # Initialize ROS node
    rospy.init_node('rgbd_processor', anonymous=True)

    # Subscribe to color and depth data topics
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.Subscriber('/camera/depth/image_raw', Image, image_callback)

    # Intrinsic parameters for Kinect v2, adjust these according to your camera
    fx = 525.0
    fy = 525.0
    cx = 319.5
    cy = 239.5
    intrinsic = o3d.camera.PinholeCameraIntrinsic(640, 480, fx, fy, cx, cy)

    # Spin ROS node
    rospy.spin()

if __name__ == '__main__':
    main()
