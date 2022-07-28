import cv2
import numpy as np

import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from .ImageNodeBase import ImageNodeBase


class MarkerDetector(ImageNodeBase):
    def __init__(self, prefix: str):
        super(MarkerDetector, self).__init__(rgb_enable=True)

        # Aruco detector parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_size = 0.018

        # Member variables
        self.inner_corner = []
        self.img2show = None
        self.new_frame = False
        # Translation of area
        self.trans = None

        self.image_publisher = self.create_publisher(Image, prefix+"/MarkerDetector/image", 3)
        self.translation_publisher = self.create_publisher(Float64MultiArray, prefix+"/MarkerDetector/translation", 3)
        timer_period = 0.04  # seconds
        self.timer = self.create_timer(timer_period, self.marker_timer_callback)

    def marker_timer_callback(self):
        translation_msg = Float64MultiArray()
        if self.rgb_flag and self.rgb_img is not None:
            self.img2show = self.rgb_img.copy()
            # Detect the markers in the image
            ret = self.detect_markers(self.rgb_img)
            if ret:
                self.new_frame = True
                translation_msg.data.append(1.0)
                translation_msg.data.extend(self.trans[0].tolist())
            else:
                translation_msg.data.append(0.0)

            # Publish
            self.image_publisher.publish(self.bridge.cv2_to_imgmsg(self.img2show, "bgr8"))
            self.rgb_flag = False
        else:
            translation_msg.data.append(0.0)

        self.translation_publisher.publish(translation_msg)

    def detect_markers(self, img):
        # Aruco markers detection
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)

        # So far, it only supports 4 corners
        if not len(corners) == 4:
            return False

        # Sort to make it in clockwise order
        corners = list(corners)
        ids = ids.flatten()
        ids, corners = zip(*sorted(zip(ids, corners)))
        ids = np.reshape(ids, (len(ids), 1))

        # Visualization
        cv2.aruco.drawDetectedMarkers(self.img2show, corners, ids)

        # Draw the rect of workspace
        # Clear list to avoid repeat recording
        markers_center = []
        for corner in corners:
            markers_center.append(np.mean(corner[0], axis=0))
        inner_corner = [corners[0][0][2], corners[1][0][3], corners[2][0][0], corners[3][0][1]]
        self.inner_corner = inner_corner
        # area_center = np.mean(np.array(markers_center), axis=0).astype(np.int32)
        # vec_center_px = [(self.markers_center[0][0]+self.markers_center[1][0])/2,
        #                  (self.markers_center[0][1]+self.markers_center[1][1])/2]
        # cv2.circle(self.image, self.center, 3, (255, 0, 0), thickness=1)
        # cv2.drawContours(self.image, [np.int0(self.markers_center)], 0, (255, 0, 0), 2)
        cv2.drawContours(self.img2show, [np.int0(inner_corner)], 0, (255, 0, 0), 2)
        # rect = cv2.minAreaRect(np.int0(self.markers_center))
        # self.rot = rect[2] / 180.0 * np.pi
        # self.rot = atan2(-vec_center_px[1] + center_px[1], vec_center_px[0] - center_px[0])

        # self.logger.info("Current rotation angle of board is:{}".format(self.rot*180.0/np.pi))

        # Detect pose
        [rvecs, tvecs, _] = cv2.aruco.estimatePoseSingleMarkers(corners, self.aruco_size,
                                                                self.camera_k, self.camera_d)
        self.trans = np.mean(tvecs, axis=0)

        return True


def main(args=None):
    rclpy.init(args=args)

    image_node_base = MarkerDetector(prefix="/test")

    rclpy.spin(image_node_base)

    # Destroy the node explicitly
    image_node_base.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
