import numpy as np
import rclpy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge, CvBridgeError
from tf2_ros import TransformBroadcaster

from DTOID.DTOIDModule import DTOIDModule
from src.franka_panda_RL.franka_perception.franka_perception.base.ImageNodeBase import ImageNodeBase
from utils.BoxDetector import BoxDetector

camera_k = np.array([[604.207275390625, 0.0, 317.8360290527344],
                     [0.0, 603.8510131835938, 234.7834014892578],
                     [0.0, 0.0, 1.0]])


class ObjDetectorNode(ImageNodeBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        """
            mode: 0 for not processing image
                  1 for detecting with object detection
                  2 for detecting box's lattice
        """
        self.step = 0

        self.rgb_img = None
        self.depth_img = None

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, "/test_out", 3)
        self.flag_publisher = self.create_publisher(Int32, "/flag", 3)
        self.step_subscription = self.create_subscription(Int32, "/step_", self.step_callback, 1)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.detector_model = DTOIDModule(template_dir="gum/output")
        self.box_detector = BoxDetector()

    def rgb_callback(self, rgb_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except CvBridgeError as e:
            self.logger.error(e)
            return

        # Used to show the processed image
        self.rgb_img = cv_image.copy()

    def depth_callback(self, depth_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except CvBridgeError as e:
            self.logger.error(e)
            return

        # Used to show the processed image
        self.depth_img = cv_image.copy()

    def timer_callback(self):
        if self.rgb_img is not None and self.depth_img is not None:
            img = self.rgb_img

            flag = Int32()
            flag.data = 0
            angel = 0
            x_y_z = np.zeros((3,))
            if self.step == 1:
            # if True:
                success, img, bbox, angel = self.detector_model.process(self.rgb_img)
                if success:
                    depth_array = self.depth_img[bbox[1]:bbox[3], bbox[0]:bbox[2]]
                    avg_depth = np.average(depth_array)/1000.0

                    bbox_center = [(bbox[0]+bbox[2])//2, (bbox[1]+bbox[3])//2]
                    u_v_1 = np.array([bbox_center[0], bbox_center[1], 1]).T
                    x_y_z = np.matmul(np.linalg.inv(camera_k), u_v_1) * avg_depth
                    # self.tf_handler(x_y_z, angel=angel)

                    flag.data = 1
            self.tf_handler(x_y_z, angel=angel)
            self.flag_publisher.publish(flag)

            x_y_z_list = [np.zeros((3,)), np.zeros((3,)), np.zeros((3,)), np.zeros((3,))]
            if self.step == 2:
            # if True:
                img, bbox_list = self.box_detector.process(self.rgb_img)
                if len(bbox_list) == 4:
                    for i, bbox in enumerate(bbox_list):
                        depth_array = self.depth_img[bbox[1]:bbox[3], bbox[0]:bbox[2]]
                        avg_depth = np.average(depth_array)/1000.0

                        bbox_center = [(bbox[0]+bbox[2])//2, (bbox[1]+bbox[3])//2]
                        u_v_1 = np.array([bbox_center[0], bbox_center[1], 1]).T
                        x_y_z_list[i] = np.matmul(np.linalg.inv(camera_k), u_v_1) * avg_depth
            for i, bbox in enumerate(x_y_z_list):
                self.tf_handler(x_y_z_list[i], child_frame="lattice_"+str(i), angel=180)

            img = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.image_publisher.publish(img)

    def tf_handler(self, x_y_z, frame_id="camera_color_optical_frame", child_frame="hand_pose", angel=0):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = x_y_z[0]
        t.transform.translation.y = x_y_z[1]
        t.transform.translation.z = x_y_z[2]

        q = get_quaternion_from_euler(0, 0, angel)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

    # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def step_callback(self, msg):
        self.step = msg.data


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)

    hand_tracker_node = ObjDetectorNode(node_name="obj_detector", rgb_enable=True, depth_enable=True)

    rclpy.spin(hand_tracker_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hand_tracker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
