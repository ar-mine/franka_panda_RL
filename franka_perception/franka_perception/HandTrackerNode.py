import numpy as np
import copy

import rclpy

from cv_bridge import CvBridge, CvBridgeError
from tf2_ros import TransformBroadcaster

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped

from src.franka_panda_RL.franka_perception.franka_perception.base.ImageNodeBase import ImageNodeBase
from HandTrackingModule import HandDetector, post_process

camera_k = np.array([[604.207275390625, 0.0, 317.8360290527344],
                     [0.0, 603.8510131835938, 234.7834014892578],
                     [0.0, 0.0, 1.0]])


class HandTrackerNode(ImageNodeBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.rgb_img = None
        self.depth_img = None

        self.hand_detector = HandDetector()
        self.bridge = CvBridge()

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.image_publisher = self.create_publisher(Image, "/test_out", 3)
        self.array_publisher = self.create_publisher(Float32MultiArray, "/"+self.node_name+"/array", 3)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.rect_size = 3
        self.img_size = (640, 480)

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
            rgb_img_ = copy.copy(self.rgb_img)
            depth_img_ = copy.copy(self.depth_img)

            img = self.hand_detector.find_hands(rgb_img_)
            lmlist = self.hand_detector.find_position(img)
            img, bbox = post_process(img, lmlist)

            if len(bbox) == 4:
                bbox_center = [(bbox[0]+bbox[1])//2, (bbox[2]+bbox[3])//2]

                # Calculate the hand average depth
                bound_x_min = bbox_center[0]-self.rect_size if bbox_center[0] >= self.rect_size else 0
                bound_x_max = bbox_center[0]+self.rect_size \
                    if bbox_center[0] <= self.img_size[0]-self.rect_size else self.img_size[0]
                bound_y_min = bbox_center[1]-self.rect_size if bbox_center[1] >= self.rect_size else 0
                bound_y_max = bbox_center[1]+self.rect_size \
                    if bbox_center[1] <= self.img_size[1]-self.rect_size else self.img_size[1]
                hand_depth = np.average(depth_img_[bound_y_min:bound_y_max, bound_x_min:bound_x_max])/1000.0

                u_v_1 = np.array([bbox_center[0], bbox_center[1], 1]).T
                ux_uy = np.matmul(np.linalg.inv(camera_k), u_v_1)
                x_y_z = hand_depth * ux_uy
                # print("%.2f, %.2f, %.2f" % (x_y_z[0], x_y_z[1], x_y_z[2]))
                float_array = Float32MultiArray()
                float_array.data = x_y_z.tolist()+ux_uy.tolist()
                self.array_publisher.publish(float_array)
                self.tf_handler(x_y_z)
                # Ensure the depth is credible
                # if hand_depth >= 0.30:
                #     u_v_1 = np.array([bbox_center[0], bbox_center[1], 1]).T
                #     ux_uy = np.matmul(np.linalg.inv(camera_k), u_v_1)
                #     x_y_z = hand_depth * ux_uy
                #     # print("%.2f, %.2f, %.2f" % (x_y_z[0], x_y_z[1], x_y_z[2]))
                #     float_array = Float32MultiArray()
                #     float_array.data = x_y_z.tolist()+ux_uy.tolist()
                #     self.array_publisher.publish(float_array)
                #     self.tf_handler(x_y_z)
                # else:
                #     # When the depth is not credible
                #     pass
            else:
                # When bbox cannot give enough values
                float_array = Float32MultiArray()
                float_array.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.array_publisher.publish(float_array)

            img = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.image_publisher.publish(img)

    def tf_handler(self, x_y_z, frame_id="camera_color_optical_frame", child_frame="hand_pose"):
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

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    hand_tracker_node = HandTrackerNode(node_name="hand_tracker", rgb_enable=True, depth_enable=True)

    rclpy.spin(hand_tracker_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hand_tracker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
