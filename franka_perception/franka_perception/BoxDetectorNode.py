import numpy as np
import rclpy
from rclpy.action import ActionClient
# from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError

from franka_interface.srv import StackPick
from franka_perception.base.ImageNodeBase import ImageNodeBase
from franka_perception.yolov5.detector import YoloV5

import time
import cv2

# 0.274459 0.00285477 0.0384874   -0.432977 0.417711 -0.569811 0.55979
T_base2camera = np.array([[0.0016669,  0.2762302,  0.9610900, 0.274459],
                          [-0.9996665, -0.0243063,  0.0087197, 0.00285477],
                          [0.0257692, -0.9607841,  0.2760976, 0.0384874],
                          [0, 0, 0, 1]])


class BoxDetectorNode(ImageNodeBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.rgb_img = None
        self.depth_img = None

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, "/detector_out", 3)

        self.detector = YoloV5()

        self.timer = self.create_timer(1/10, self.timer_callback)

        self.bbox = []
        self.stack_pick_srv = self.create_service(StackPick, 'stack_pick', self.stack_pick_callback)

        self.last_time = time.time()

    def timer_callback(self):
        if self.rgb_img is not None and self.depth_img is not None:
            img = self.rgb_img
            img, det = self.detector.inference(img)
            det = det.cpu().numpy().astype(int)

            cv2.putText(img, 'FPS:%.2f' % (1/(time.time() - self.last_time)), (0, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1, color=(0, 0, 255), thickness=1)

            bbox = []
            for *xyxy, conf, cls in reversed(det):
                if cls != 0:
                    continue
                center_xy = [(xyxy[0]+xyxy[2])//2, (xyxy[1]+xyxy[3])//2]

                temp = xyxy
                temp.extend(center_xy)
                bbox.append(temp)

            self.bbox = sorted(bbox, reverse=True, key=lambda element: element[1])
            for i, l in enumerate(self.bbox):
                cv2.putText(img, str(i), l[4:], cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=1, color=(0, 0, 255), thickness=3)

            img = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.image_publisher.publish(img)

            self.last_time = time.time()

    def stack_pick_callback(self, request, response):
        idx = request.idx
        bbox = self.bbox
        len_bbox = len(bbox)
        if idx >= len_bbox:
            response.last_one = -1
            response.target_pose = Pose()
            self.get_logger().info("Wrong idx number!")

        # Generate graph array
        d_graph = np.zeros([len_bbox, len_bbox])
        for i in range(len_bbox):
            for j in range(i+1, len_bbox):
                ret = collision_check(bbox[i], bbox[j])
                if ret == 1:
                    d_graph[i, j] = 1
                elif ret == 2:
                    d_graph[j, i] = 1
        print(d_graph)

        response.last_one = 1
        while not np.sum(d_graph[:, idx]) == 0:
            response.last_one = 0
            idx = np.where(d_graph[:, idx] == 1)[0][0]

        response.target_pose = self.camera2world(bbox[idx])
        return response

    def camera2world(self, xyxy):
        center_xy = [(xyxy[0] + xyxy[2]) // 2, (xyxy[1] + xyxy[3]) // 2]
        top_xy = [(xyxy[0] + xyxy[2]) // 2, xyxy[1]]
        depth = np.sum(self.depth_img[center_xy[1] - 2:center_xy[1] + 3, center_xy[0] - 2:center_xy[0] + 3]) / 25 / 1000

        u_v_1 = np.array([top_xy[0], top_xy[1], 1]).T
        x_y_z = np.matmul(np.linalg.inv(self.camera_k), u_v_1) * (depth + 0.01)

        T_camera2target = np.eye(4)
        T_camera2target[:3, 3] = x_y_z
        T_camera2target = np.matmul(T_base2camera, T_camera2target)[:3, 3]
        T_camera2target[0] += 0.05

        pose = Pose()
        pose.position.x = T_camera2target[0]
        pose.position.y = T_camera2target[1]
        pose.position.z = T_camera2target[2]

        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
        return pose


def collision_check(xyxy_a, xyxy_b):
    # Check intersection
    ret = 0
    minx = max(xyxy_a[0], xyxy_b[0])
    miny = max(xyxy_a[1], xyxy_b[1])
    maxx = min(xyxy_a[2], xyxy_b[2])
    maxy = min(xyxy_a[3], xyxy_b[3])
    if not(minx > maxx+2 or miny > maxy+2):
        ret = 1

    if ret:
        if xyxy_a[3] < xyxy_b[3]:
            ret = 1
        elif xyxy_a[3] > xyxy_b[3]:
            ret = 2
        elif xyxy_a[3] == xyxy_b[3]:
            ret = 3
        else:
            ret = -1
    return ret


def main(args=None):
    rclpy.init(args=args)

    box_detector_node = BoxDetectorNode(node_name="box_detector", rgb_enable=True, depth_enable=True)

    rclpy.spin(box_detector_node)

    box_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
