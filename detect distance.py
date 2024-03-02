                                                        # Nefunguje (No device connected) 
import cv2
import numpy as np
import pyrealsense2 as rs

class RealSenseCamera:
    def __init__(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)

    def get_frames(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return depth_image, color_image

    def release(self):
        # Stop streaming
        self.pipeline.stop()


def main():
    # Create RealSense camera object
    camera = RealSenseCamera()

    try:
        while True:
            # Get depth and color frames
            depth_frame, color_frame = camera.get_frames()

            # Display the frames
            cv2.imshow('Depth Frame', depth_frame)
            cv2.imshow('Color Frame', color_frame)

            # Break the loop if 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Release the camera resources
        camera.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
