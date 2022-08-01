import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from math import atan2

import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from base.MarkerDetector import MarkerDetector


class ShapeDetector(MarkerDetector):
    def __init__(self):
        super(ShapeDetector, self).__init__(prefix="/ShapeDetector")

        self.box_trans = []
        self.box_rot = []
        self.w_l = []

        self.shape_image_publisher = self.create_publisher(Image, "/ShapeDetector/image", 3)
        self.shape_info_publisher = self.create_publisher(Float64MultiArray, "/ShapeDetector/box_info", 3)

        timer_period = 0.05  # seconds
        self.shape_timer = self.create_timer(timer_period, self.shape_timer_callback)

    def shape_timer_callback(self):
        box_info_msg = Float64MultiArray()
        if self.img2show is not None:
            if self.new_frame:
                self.detect_shapes(self.rgb_img)

                if self.box_rot and self.box_trans:
                    #   0                   1-3             4-7                     8-9                 9*i+x
                    #   no. of markers      translation     rotation(quaternion)    width, length
                    box_info_msg.data.append(len(self.box_rot))
                    for i in range(len(self.box_rot)):
                        box_info_msg.data.extend(self.box_trans[i])
                        box_info_msg.data.extend(self.box_rot[i])
                        box_info_msg.data.extend(self.w_l[i])
                        # self.logger.info("Current ratio: %f" % (self.w_l[i][1]/self.w_l[i][0]))
                else:
                    box_info_msg.data.append(0.0)

                self.new_frame = False
            else:
                box_info_msg.data.append(0.0)
            # Publish
            self.shape_image_publisher.publish(self.bridge.cv2_to_imgmsg(self.img2show, "bgr8"))
        else:
            box_info_msg.data.append(0.0)
        self.shape_info_publisher.publish(box_info_msg)

    def detect_shapes(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Filter the area outside
        blank = np.ones(gray.shape)*255
        cv2.fillPoly(blank, pts=[np.int0(self.inner_corner)], color=0)
        gray[blank == 255] = 255
        gray = 255 - gray

        # Enhance contrast
        # gray_temp = 2 * gray
        # gray_temp[gray_temp > 255] = 255
        gray[gray >= 240] = 0
        gray = gray.astype(np.uint8)

        # # Threshold segmentation
        # thresh_inv = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
        thresh_inv = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)[1]
        # # Blur the image
        # blur = cv2.GaussianBlur(thresh_inv, (1, 1), 0)
        # thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

        # Find contours
        contours = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        # Find boxes
        box_position = []
        box_rot = []
        w_l = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            if w * h < 800 or w * h > 40000 or w < 60 or h < 60:
                continue
            elif y == 0:
                continue

            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box_int = np.int0(box)

            length = [box[1, :]-box[0, :], box[2, :]-box[1, :]]
            # vect = [box[1, :]-box[0, :], box[2, :]-box[1, :], box[3, :]-box[2, :], box[0, :]-box[3, :]]
            length_norm = np.linalg.norm(length, axis=1)
            w_l.append(np.sort(length_norm).tolist())
            vec = length[np.argmax(length_norm)]
            # cv2.arrowedLine(self.image, np.int0(rect[0]), np.int0((box[0] + box[1]) / 2), (255, 255, 255), 1)
            # cv2.arrowedLine(self.image, np.int0(rect[0]), np.int0((box[0] + box[3]) / 2), (255, 255, 255), 1)

            position = self.trans[0][2] * np.matmul(np.linalg.inv(self.camera_k), np.append(np.mean(box, axis=0), 1.0))

            box_position.append(position)
            rot_z = atan2(vec[1], vec[0])
            box_rot.append(R.from_euler('z', rot_z).as_quat())
            # box_rot.append(R.from_euler('z', (rect[2] + 90.0) / 180.0 * np.pi).as_quat())

            # using drawContours() function
            cv2.drawContours(self.img2show, [box_int], 0, (0, 0, 255), 2)
            # cv2.putText(self.image,'Pos A', (min(x_ls),min(y_ls)), cv2.FONT_HERSHEY_PLAIN, 3, (255,0,255), 3)
        self.box_trans = box_position
        self.box_rot = box_rot
        self.w_l = w_l


def main(args=None):
    rclpy.init(args=args)

    image_node_base = ShapeDetector()

    rclpy.spin(image_node_base)

    # Destroy the node explicitly
    image_node_base.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
