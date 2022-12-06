import numpy as np
import rclpy
import copy
from rclpy.action import ActionClient
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
import open3d as o3d

from franka_interface.srv import StackPick
from franka_perception.base.ImageNodeBase import ImageNodeBase
from franka_perception.yolov5.detector import YoloV5

import time
import cv2

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

        self.detector = YoloV5()

        self.timer = self.create_timer(1 / 10, self.timer_callback)

        self.bbox = []
        self.stack_pick_srv = self.create_service(StackPick, 'stack_pick', self.stack_pick_callback)

        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            640, 480, 616.0755615234375, 616.6409912109375, 335.7129211425781, 234.61709594726562)
        self.pcd_publisher = self.create_publisher(PointCloud2, "/pcd", 1)

        self.last_time = time.time()

    def timer_callback(self):
        if self.rgb_img is not None and self.depth_img is not None:
            img_rgb = copy.copy(self.rgb_img)
            img_depth = copy.copy(self.depth_img)

            # Annotate bounding boxes
            img, det = self.detector.inference(copy.copy(img_rgb))
            det = det.cpu().numpy().astype(int)
            # Get main result
            bbox = []
            for *xyxy, conf, cls in reversed(det):
                if cls != 0:
                    continue
                center_xy = [(xyxy[0] + xyxy[2]) // 2, (xyxy[1] + xyxy[3]) // 2]

                temp = xyxy
                temp.extend(center_xy)
                bbox.append(temp)
            # Label each box
            self.bbox = sorted(bbox, reverse=True, key=lambda element: element[1])
            for i, l in enumerate(self.bbox):
                cv2.putText(img, str(i), l[4:], cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=1, color=(0, 0, 255), thickness=3)

            # Segment box
            pcd_list = []
            for b in bbox:
                pcd_list.extend(self.segment(b, img_rgb, img_depth))
                img_rgb[b[1]:b[3], b[0]:b[2], :] = 0
                img_depth[b[1]:b[3], b[0]:b[2]] = 0

            pcd_list.append(self.o3d_pcd2numpy(self.rgbd_image2pcd(img_rgb, img_depth)))
            pcd = np.concatenate(pcd_list, axis=0)
            pcd_msg = self.np_pcd2ros_msg(pcd, "camera_depth_optical_frame")

            cv2.putText(img, 'FPS:%.2f' % (1 / (time.time() - self.last_time)), (0, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=1, color=(0, 0, 255), thickness=1)
            img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            # Publish annotated images and segmented point cloud
            self.image_publisher.publish(img_msg)
            self.pcd_publisher.publish(pcd_msg)

            # Update time recorded
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

    def segment(self, b, img_rgb, img_depth):
        result_rgb = np.zeros_like(img_rgb)
        result_depth = np.zeros_like(img_depth)
        result_rgb[b[1]:b[3], b[0]:b[2], :] = img_rgb[b[1]:b[3], b[0]:b[2], :]
        result_depth[b[1]:b[3], b[0]:b[2]] = img_depth[b[1]:b[3], b[0]:b[2]]
        result_rgb_o3d = o3d.geometry.Image(result_rgb.astype(np.uint8))
        result_depth_o3d = o3d.geometry.Image(result_depth.astype(np.uint16))
        result_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            result_rgb_o3d, result_depth_o3d)
        result_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(result_rgbd, self.intrinsic)

        plane_model, inliers = result_pcd.segment_plane(distance_threshold=0.01,
                                                        ransac_n=3,
                                                        num_iterations=100)

        inlier_cloud = result_pcd.select_by_index(inliers)
        inlier_cloud.paint_uniform_color([1.0, 0, 0])
        outlier_cloud = result_pcd.select_by_index(inliers, invert=True)

        return [self.o3d_pcd2numpy(inlier_cloud), self.o3d_pcd2numpy(outlier_cloud)]

    """********************* Functions for convert ***************************"""
    def rgbd_image2pcd(self, img_rgb, img_depth):
        """
        Convert color and depth image (numpy array format) to point cloud (open3D format)
        """
        color_raw = o3d.geometry.Image(img_rgb.astype(np.uint8))
        depth_raw = o3d.geometry.Image(img_depth.astype(np.uint16))
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, self.intrinsic)
        return pcd

    @staticmethod
    def o3d_pcd2numpy(o3d_pcd: o3d.geometry.PointCloud()):
        """
            Convert point cloud (open3D format) to numpy array (xyzrgb for ROS PointCloud2 format)
        """
        points = np.asarray(o3d_pcd.points)
        colors = np.asarray(o3d_pcd.colors)
        np_array = np.concatenate([points, colors], axis=1)
        return np_array

    def np_pcd2ros_msg(self, points, parent_frame):
        """ Creates a point cloud message.
        Args:
            points: Nx6 array of xyz positions (m) and rgb colors (0..1)
            parent_frame: frame in which the point cloud is defined
        Returns:
            sensor_msgs/PointCloud2 message
        """

        item_size = np.dtype(np.float32).itemsize

        data = points.astype(np.float32).tobytes()

        fields = [PointField(
            name=n, offset=i * item_size, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate('xyzrgb')]

        header = Header(frame_id=parent_frame, stamp=self.get_clock().now().to_msg())

        return PointCloud2(
            header=header,
            height=1,
            width=points.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(item_size * 6),
            row_step=(item_size * 6 * points.shape[0]),
            data=data
        )


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
