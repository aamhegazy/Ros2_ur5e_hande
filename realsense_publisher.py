import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('realsense_publisher')
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info('RealSense publisher started')

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if color_frame:
            color_img = np.asanyarray(color_frame.get_data())
            self.color_pub.publish(self.bridge.cv2_to_imgmsg(color_img, 'bgr8'))
        if depth_frame:
            depth_img = np.asanyarray(depth_frame.get_data())
            self.depth_pub.publish(self.bridge.cv2_to_imgmsg(depth_img, '16UC1'))

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main():
    rclpy.init()
    node = RealSensePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
