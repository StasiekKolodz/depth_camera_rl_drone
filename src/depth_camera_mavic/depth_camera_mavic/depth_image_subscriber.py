import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class DepthImageSubscriber(Node):

    def __init__(self):
        super().__init__('depth_image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/Depth_Mavic_2_PRO/range_finder/image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.camera_range = 10
        self.depth_image_shape = (64, 64, 1)

    def listener_callback(self, msg):
        self.get_logger().info('Receiving depth image')
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.current_frame = self.clip_depth_image(self.current_frame, self.camera_range)
            cv2.imshow("Depth Image", self.current_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")
    def clip_depth_image(self, image, camera_range):
        # Clip image and rescale values to [0,255] range
        image = np.clip(image, 0, camera_range) / camera_range * 255
        return image.reshape(self.depth_image_shape[0], self.depth_image_shape[1], 1).astype(np.uint8)

def main(args=None):
    rclpy.init(args=args)
    depth_image_subscriber = DepthImageSubscriber()
    rclpy.spin(depth_image_subscriber)
    depth_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()