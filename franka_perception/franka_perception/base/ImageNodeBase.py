import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError


class ImageNodeBase(Node):
    """
    Param list:
        node_name
        rgb_enable
            rgb_topic_name
        depth_enable
            depth_topic_name
    """
    def __init__(self, **kwargs):
        # Create ROS2 node
        if 'node_name' not in kwargs.keys():
            self.node_name = "image_node_base"
        else:
            self.node_name = kwargs['node_name']
        super().__init__(self.node_name)

        # Create RGB subscription
        if 'rgb_enable' in kwargs.keys() and kwargs['rgb_enable'] is not False:
            self.rgb_enable = True
            if 'rgb_topic_name' not in kwargs.keys():
                self.rgb_topic_name = "/camera/color/image_raw"
            else:
                self.rgb_topic_name = kwargs['rgb_topic_name']
            self.rgb_subscription = self.create_subscription(Image, self.rgb_topic_name, self.rgb_callback, 3)

            self.rgb_img = None
            self.rgb_flag = False
        else:
            self.rgb_enable = False

        # Create Depth subscription
        if 'depth_enable' in kwargs.keys() and kwargs['depth_enable'] is not False:
            self.depth_enable = True
            if 'depth_topic_name' not in kwargs.keys():
                self.depth_topic_name = "/camera/aligned_depth_to_color/image_raw"
            else:
                self.depth_topic_name = kwargs['depth_topic_name']
            self.depth_subscription = self.create_subscription(Image, self.depth_topic_name, self.depth_callback, 3)

            self.depth_img = None
            self.depth_flag = False
        else:
            self.depth_enable = False

        self.logger = self.get_logger()
        self.bridge = CvBridge()

        # Camera info
        self.cameraInfo_subscription = self.create_subscription(CameraInfo, "/camera/color/camera_info",
                                                                self.camera_info_callback, 3)
        self.camera_k = None
        self.camera_d = None
        self.width_height = [0, 0]

    def rgb_callback(self, rgb_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except CvBridgeError as e:
            self.logger.error(e)
            return
        self.rgb_img = cv_image.copy()
        self.rgb_flag = True

    def depth_callback(self, depth_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except CvBridgeError as e:
            self.logger.error(e)
            return
        self.depth_img = cv_image.copy()
        self.depth_flag = True

    def camera_info_callback(self, camera_info_msg):
        if self.camera_k is None or self.camera_d is None:
            self.camera_k = np.array(camera_info_msg.k).reshape((3, 3))
            self.camera_d = np.array(camera_info_msg.d)
            self.width_height = [camera_info_msg.width, camera_info_msg.height]


def main(args=None):
    rclpy.init(args=args)

    image_node_base = ImageNodeBase()

    rclpy.spin(image_node_base)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_node_base.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
