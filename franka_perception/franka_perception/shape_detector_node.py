import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, JointState
from visualization_msgs.msg import Marker, MarkerArray

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import threading
import json
from math import atan2

from scipy.spatial.transform import Rotation as R


def config_parser():
    import configargparse
    parser = configargparse.ArgumentParser()
    parser.add_argument('--config', is_config_file=True,
                        help='config file path', default="./configs/default.txt")

    parser.add_argument("--node", type=str,
                        help='Node name')
    # Topic name
    parser.add_argument("--img_topic_in", type=str,
                        help='The input RGB image topic name')
    parser.add_argument("--img_topic_out", type=str, default='/detected_shape',
                        help='The topic outputs processed images')
    parser.add_argument("--markers_topic", type=str, default='/markers',
                        help='The topic publish markers')

    return parser


def quaternion_multiply(quaternion1, quaternion0):
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return np.array([x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
                     -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0, ], dtype=np.float64)


class ShapeDetectorNode(Node):

    def __init__(self):
        parser = config_parser()
        args = parser.parse_args()
        super().__init__(args.node)

        img_topic_in = args.img_topic_in
        img_topic_out = args.img_topic_out
        markers_topic = args.markers_topic

        self.image_publisher = self.create_publisher(Image, img_topic_out, 3)
        self.image_subscription = self.create_subscription(Image, img_topic_in, self.image_callback, 3)
        self.markers_publisher = self.create_publisher(MarkerArray, markers_topic, 1)
        self.joint_states_subscription = self.create_subscription(JointState, "/joint_states",
                                                                  self.joint_states_callback, 1)

        self.logger = rclpy.logging.get_logger(args.node)

        self.bridge = CvBridge()
        self.lock = threading.RLock()

        # Camera parameters
        self.camera_k = np.array([[906.3109741210938, 0.0, 636.7540283203125],
                                  [0.0, 905.7764892578125, 352.17510986328125],
                                  [0.0, 0.0, 1.0]
                                  ])
        self.camera_d = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # Aruco detector parameter
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_size = 0.018

        # Image temp to be published
        self.image = None
        self.markers_center = []
        self.inner_corner = []
        self.markers = MarkerArray()

        # Pose recorded for saving
        self.trans = []
        self.rot = 0
        self.box_trans = []
        self.box_rot = []

        self.rvecs = None
        self.tvecs = None
        timer_period = 0.04  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter("record_flag", 0)

        self.joint_states = None
        self.step = 0

    def joint_states_callback(self, joint_states_msg):
        with self.lock:
            self.joint_states = joint_states_msg

    def timer_callback(self):
        self.markers.markers.clear()
        with self.lock:
            if self.rvecs is not None and self.tvecs is not None:
                rvecs = self.rvecs
                tvecs = self.tvecs
            else:
                rvecs = []
                tvecs = []

        for i in range(len(rvecs)):
            tvec = tvecs[i][0]
            rvec = R.from_euler('xyz', rvecs[i][0]).as_quat()
            self.markers.markers.append(self.marker_gen('camera_color_optical_frame',
                                                        'markers',
                                                        i,
                                                        (255.0, 255.0, 255.0),
                                                        tvec,
                                                        (0.0, 0.0, 0.0, 1.0),
                                                        s_type=Marker.CUBE,
                                                        scale=(0.05, 0.05, 0.01)))

        with self.lock:
            if self.box_trans:
                box_position = self.box_trans
                box_rot = self.box_rot
            else:
                box_position = []
                box_rot = []
        for i, p in enumerate(box_position):
            self.markers.markers.append(self.marker_gen('camera_color_optical_frame',
                                                        'boxes',
                                                        i,
                                                        (0.0, 255.0, 128.0),
                                                        p,
                                                        box_rot[i],
                                                        s_type=Marker.CUBE,
                                                        scale=(0.05, 0.02, 0.03)))

        self.markers_publisher.publish(self.markers)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.logger.error(e)
            return

        # Used to show the processed image
        self.image = cv_image.copy()

        # Add markers' pose
        ret = self.detect_markers(cv_image)
        if ret != 0:
            # Add boxes' pose and rects
            self.detect_shapes(cv_image)

            # Query the 'record' flag
            record_flag = self.get_parameter('record_flag').get_parameter_value().integer_value
            if record_flag:
                if record_flag == 1:
                    self.scene_save("demo_data.json")
                    self.step = 1
                elif record_flag == 2:
                    self.scene_save("repo_data.json")
                    self.step = 2
                # Reset flag to avoid endless loop
                new_param = rclpy.parameter.Parameter(
                    'record_flag',
                    rclpy.Parameter.Type.INTEGER,
                    0
                )
                self.set_parameters([new_param])

        img_msg_pub = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
        self.image_publisher.publish(img_msg_pub)

    def detect_markers(self, img):
        # Aruco markers detection
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img,
                                                           self.aruco_dict,
                                                           parameters=self.aruco_params)

        # So far, it only supports 4 corners
        if not len(corners) == 4:
            return 0

        # Sort to make it in clockwise order
        corners = list(corners)
        ids = ids.flatten()
        ids, corners = zip(*sorted(zip(ids, corners)))
        ids = np.reshape(ids, (len(ids), 1))

        # Visualization
        cv2.aruco.drawDetectedMarkers(self.image, corners, ids)

        # Draw the rect of workspace
        # Clear list to avoid repeat recording
        self.markers_center.clear()
        self.inner_corner.clear()

        for corner in corners:
            self.markers_center.append(np.mean(corner[0], axis=0))
        self.inner_corner = [corners[0][0][2], corners[1][0][3], corners[2][0][0], corners[3][0][1]]
        center_px = np.mean(np.array(self.markers_center), axis=0).astype(np.int32)
        vec_center_px = [(self.markers_center[0][0]+self.markers_center[1][0])/2,
                         (self.markers_center[0][1]+self.markers_center[1][1])/2]
        # cv2.circle(self.image, self.center, 3, (255, 0, 0), thickness=1)
        # cv2.drawContours(self.image, [np.int0(self.markers_center)], 0, (255, 0, 0), 2)
        cv2.drawContours(self.image, [np.int0(self.inner_corner)], 0, (255, 0, 0), 2)
        # rect = cv2.minAreaRect(np.int0(self.markers_center))
        # self.rot = rect[2] / 180.0 * np.pi
        self.rot = atan2(-vec_center_px[1] + center_px[1], vec_center_px[0] - center_px[0])

        # self.logger.info("Current rotation angle of board is:{}".format(self.rot*180.0/np.pi))

        # Detect pose
        [self.rvecs, self.tvecs, _] = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_size,
                                                                          self.camera_k, self.camera_d)
        self.trans = np.mean(self.tvecs, axis=0)

        return 1

    def detect_shapes(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        blank = np.zeros(gray.shape)
        cv2.fillPoly(blank, pts=[np.int0(self.inner_corner)], color=1)
        gray[blank == 0] = 255

        gray_temp = 2.5 * gray
        gray_temp[gray_temp > 255] = 255
        gray_temp = np.round(gray_temp)
        gray = gray_temp.astype(np.uint8)

        thresh_inv = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        # Blur the image
        blur = cv2.GaussianBlur(thresh_inv, (1, 1), 0)
        thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

        # find contours
        contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]

        self.box_trans.clear()
        self.box_rot.clear()
        box_position = []
        box_rot = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w * h < 800 or w * h > 40000:
                continue
            elif y == 0:
                continue

            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box_int = np.int0(box)

            vect = [box[1, :]-box[0, :], box[2, :]-box[1, :], box[3, :]-box[2, :], box[0, :]-box[3, :]]
            vect_norm = np.linalg.norm(vect, axis=1)
            vec = vect[np.argmax(vect_norm)]
            # cv2.arrowedLine(self.image, np.int0(rect[0]), np.int0((box[0] + box[1]) / 2), (255, 255, 255), 1)
            # cv2.arrowedLine(self.image, np.int0(rect[0]), np.int0((box[0] + box[3]) / 2), (255, 255, 255), 1)

            position = self.trans[0][2] * np.matmul(np.linalg.inv(self.camera_k), np.append(np.mean(box, axis=0), 1.0))

            box_position.append(position)
            rot_z = atan2(vec[1], vec[0])
            if self.step == 0:
                if rot_z > 0:
                    rot_z -= np.pi
            elif self.step == 1:
                if rot_z > 0:
                    rot_z -= np.pi
            box_rot.append(R.from_euler('z', rot_z).as_quat())
            # box_rot.append(R.from_euler('z', (rect[2] + 90.0) / 180.0 * np.pi).as_quat())

            # using drawContours() function
            cv2.drawContours(self.image, [box_int], 0, (0, 0, 255), 2)
            # cv2.putText(self.image,'Pos A', (min(x_ls),min(y_ls)), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,255), 3)
        self.box_trans = box_position
        self.box_rot = box_rot

    def marker_gen(self, frame_id, class_name, idx,
                   color, pos, ori,
                   s_type=Marker.SPHERE, scale=(0.1, 0.1, 0.1)):
        # Generate the needed markers with parameters
        marker = Marker()

        marker.header.stamp = (self.get_clock().now() - Duration(nanoseconds=5e7)).to_msg()
        marker.header.frame_id = frame_id
        marker.ns = class_name
        marker.id = idx
        marker.type = s_type
        marker.action = Marker.ADD

        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = pos[2]

        marker.pose.orientation.x = ori[0]
        marker.pose.orientation.y = ori[1]
        marker.pose.orientation.z = ori[2]
        marker.pose.orientation.w = ori[3]

        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]

        marker.color.a = 0.5
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]

        return marker

    def scene_save(self, file_name: str):
        if len(self.box_trans) != len(self.box_rot):
            self.logger.info("The length of translation mismatch that of rotation, please check!")
            return
        if self.joint_states is None:
            return
        joint_positions = self.joint_states.position.tolist()[:7]
        box_num = len(self.box_trans)
        # Calculate relative translation and rotation
        relative_trans = (np.array(self.box_trans) - self.trans).tolist()
        relative_rot = [quaternion_multiply(r, R.from_euler('z', self.rot).inv().as_quat()).tolist()
                        for r in self.box_rot]
        data = {
            'num': box_num,
            'translation': relative_trans,
            'rotation': relative_rot,
            'joint_positions': joint_positions
        }
        json_string = json.dumps(data)
        with open('/home/armine/ROS2/franka_ws/src/franka_panda_RL/franka_control/'+file_name, 'w') as outfile:
            json.dump(json_string, outfile)
        self.logger.info("Save scene successful!")


def main(args=None):
    rclpy.init(args=args)

    shape_detector_node = ShapeDetectorNode()

    rclpy.spin(shape_detector_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    shape_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
