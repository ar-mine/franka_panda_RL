import numpy as np
import rclpy
import copy
import time

from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster

import open3d as o3d
import cv2

from franka_interface.srv import StackPick
from franka_perception.utils import from_two_vectors, matrix2quat
from franka_perception.o3d_utils import o3d_pcd2numpy, rgbd_image2pcd
from franka_perception.ros_utils import np2tf_msg, np_pcd2ros_msg
from franka_perception.base.ImageNodeBase import ImageNodeBase
from yolov5.detect_once import YoloDetector


# 0.274459 0.00285477 0.0384874   -0.432977 0.417711 -0.569811 0.55979
T_base2camera = np.array([[0.0016669, 0.2762302, 0.9610900, 0.274459],
                          [-0.9996665, -0.0243063, 0.0087197, 0.00285477],
                          [0.0257692, -0.9607841, 0.2760976, 0.0384874],
                          [0, 0, 0, 1]])


class BoxDetectorNode(ImageNodeBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.rgb_img = None
        self.depth_img = None

        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, "/detector_out", 3)

        self.detector = YoloDetector('box_detection')

        self.timer = self.create_timer(1 / 10, self.timer_callback)

        self.bbox = []
        self.stack_pick_srv = self.create_service(StackPick, 'stack_pick', self.stack_pick_callback)

        self.target_idx = 0
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic()
        self.pcd_publisher = self.create_publisher(PointCloud2, "/pcd", 1)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = time.time()

    def camera_info_callback(self, camera_info_msg):
        if self.camera_k is None or self.camera_d is None:
            self.camera_k = np.array(camera_info_msg.k).reshape((3, 3))
            self.camera_d = np.array(camera_info_msg.d)
            self.width_height = [camera_info_msg.width, camera_info_msg.height]
            self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
                camera_info_msg.width, camera_info_msg.height,
                camera_info_msg.k[0], camera_info_msg.k[4], camera_info_msg.k[2], camera_info_msg.k[5])

    def timer_callback(self):
        # Guarantee computation with correct intrinsic
        if self.camera_k is None or self.camera_d is None:
            return

        if self.rgb_img is not None and self.depth_img is not None:
            img_rgb = copy.deepcopy(self.rgb_img)
            img_depth = copy.deepcopy(self.depth_img)

            """>>>>>>>>>>>> RGB image processing <<<<<<<<<<<<"""
            # Annotate bounding boxes
            img, det = self.detector.inference(copy.copy(img_rgb))
            det = det.cpu().numpy().astype(int)
            # Retain detected results with 'box' class
            xyxy_center = []
            for *xyxy, conf, cls in reversed(det):
                if cls != 0:
                    continue
                center_xy = [(xyxy[0] + xyxy[2]) // 2, (xyxy[1] + xyxy[3]) // 2]

                xyxy_center.append([*xyxy, *center_xy])
            # Label each box (rule: left-bottom point, from large to small)
            self.bbox = sorted(xyxy_center, reverse=True, key=lambda element: element[1])
            for i, l in enumerate(self.bbox):
                cv2.putText(img, str(i), l[4:], cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=1, color=(0, 0, 255), thickness=3)
            cv2.putText(img, 'FPS:%.2f' % (1 / (time.time() - self.last_time)), (0, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1, color=(0, 0, 255), thickness=1)
            # Update time recorded
            self.last_time = time.time()
            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            """>>>>>>>>>>>> RGB image processing <<<<<<<<<<<<"""

            """>>>>>>>>>>>> Point Cloud processing <<<<<<<<<<<<"""
            # Segment the plane of box
            if len(self.bbox) > self.target_idx:
                b = self.bbox[self.target_idx]
                pcd_list = self.segment(img_rgb, img_depth, b)
                # Remove area marked
                img_rgb[b[1]:b[3], b[0]:b[2], :] = 0
                img_depth[b[1]:b[3], b[0]:b[2]] = 0

                pcd_list.append(o3d_pcd2numpy(rgbd_image2pcd(img_rgb, img_depth, self.intrinsic)))
                pcd = np.concatenate(pcd_list, axis=0)
                pcd_msg = np_pcd2ros_msg(pcd, self.get_clock().now().to_msg(), "camera_color_optical_frame")
            else:
                pcd_msg = None
            """>>>>>>>>>>>> Point Cloud processing <<<<<<<<<<<<"""

            # Publish annotated images and segmented point cloud
            self.image_publisher.publish(img_msg)
            if pcd_msg is not None:
                self.pcd_publisher.publish(pcd_msg)

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
            for j in range(i + 1, len_bbox):
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

    def segment(self, img_rgb, img_depth, bbox):
        result_rgb = np.zeros_like(img_rgb)
        result_depth = np.zeros_like(img_depth)
        result_rgb[bbox[1]:bbox[3], bbox[0]:bbox[2], :] = img_rgb[bbox[1]:bbox[3], bbox[0]:bbox[2], :]
        result_depth[bbox[1]:bbox[3], bbox[0]:bbox[2]] = img_depth[bbox[1]:bbox[3], bbox[0]:bbox[2]]
        # Generate point cloud from new RGB-D image
        result_pcd = rgbd_image2pcd(result_rgb, result_depth, self.intrinsic)

        plane_model, inliers = result_pcd.segment_plane(distance_threshold=0.01,
                                                        ransac_n=3,
                                                        num_iterations=100)

        inlier_cloud = result_pcd.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = result_pcd.select_by_index(inliers, invert=True)

        # Calculate box's pose
        plane_pd = o3d_pcd2numpy(inlier_cloud)
        depth = plane_pd[:, 2].sum()/plane_pd.shape[0]
        center_xy = [(bbox[0] + bbox[2]) // 2, (bbox[1] + bbox[3]) // 2]
        u_v_1 = np.array([center_xy[0], center_xy[1], 1.0]).T
        x_y_z = np.matmul(np.linalg.inv(self.camera_k), u_v_1)*depth
        x_y_z[2] = -(plane_model[0] * x_y_z[0] + plane_model[1] * x_y_z[1] + plane_model[3]) / plane_model[2]
        p_init = np.array([0, 0, 1]).transpose()
        p_goal = np.array([plane_model[0], plane_model[1], plane_model[2]]).transpose()
        R_quat = matrix2quat(from_two_vectors(p_init, p_goal))
        self.tf_broadcaster.sendTransform(np2tf_msg([*x_y_z, *R_quat], self.get_clock().now().to_msg(),
                                                    'camera_color_optical_frame', 'box_pose'))

        return [plane_pd, o3d_pcd2numpy(outlier_cloud)]


def collision_check(xyxy_a, xyxy_b):
    # Check intersection
    ret = 0
    minx = max(xyxy_a[0], xyxy_b[0])
    miny = max(xyxy_a[1], xyxy_b[1])
    maxx = min(xyxy_a[2], xyxy_b[2])
    maxy = min(xyxy_a[3], xyxy_b[3])
    if not (minx > maxx + 2 or miny > maxy + 2):
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
