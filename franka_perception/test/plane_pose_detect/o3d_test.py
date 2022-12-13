import numpy as np
import cv2 as cv
import open3d as o3d
from yolov5.detect_once import YoloDetector
from franka_perception.utils import from_two_vectors
from franka_perception.o3d_utils import rgbd_image2pcd

intrinsic = o3d.camera.PinholeCameraIntrinsic(
        640, 480, 616.0755615234375, 616.6409912109375, 335.7129211425781, 234.61709594726562)

K = np.array([[616.0755615234375, 0.0, 335.7129211425781],
              [0.0, 616.6409912109375, 234.61709594726562],
              [0.0, 0.0, 1.0]])
detector = YoloDetector()

img_rgb = cv.imread("01329_rgb.png")[..., ::-1]
img_depth = cv.imread("01329_depth.png", cv.IMREAD_UNCHANGED)
img, det = detector.inference(img_rgb.astype(float))
det = det.cpu().numpy().astype(int)
bbox = []
for *xyxy, conf, cls in reversed(det):
    if cls != 0:
        continue
    center_xy = [(xyxy[0]+xyxy[2])//2, (xyxy[1]+xyxy[3])//2]

    temp = xyxy
    temp.extend(center_xy)
    bbox.append(temp)

"""******************************"""
pcd_list = []
for b in bbox:
    result_rgb = np.zeros_like(img_rgb)
    result_depth = np.zeros_like(img_depth)
    result_rgb[b[1]:b[3], b[0]:b[2], :] = img_rgb[b[1]:b[3], b[0]:b[2], :]
    result_depth[b[1]:b[3], b[0]:b[2]] = img_depth[b[1]:b[3], b[0]:b[2]]
    img_rgb[b[1]:b[3], b[0]:b[2], :] = 0
    img_depth[b[1]:b[3], b[0]:b[2]] = 0

    result_pcd = rgbd_image2pcd(result_rgb, result_depth, intrinsic)

    plane_model, inliers = result_pcd.segment_plane(distance_threshold=0.01,
                                                     ransac_n=3,
                                                     num_iterations=1000)

    inlier_cloud = result_pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = result_pcd.select_by_index(inliers, invert=True)

    center_xy = [(b[0] + b[2]) // 2, (b[1] + b[3]) // 2]
    u_v_1 = np.array([center_xy[0], center_xy[1], 1]).T
    x_y_z = np.matmul(np.linalg.inv(K), u_v_1)
    x_y_z[2] = -(plane_model[0]*x_y_z[0] + plane_model[1]*x_y_z[1] + plane_model[3])/plane_model[2]
    p_init = np.array([0, 0, 1]).transpose()
    p_goal = np.array([plane_model[0], plane_model[1], plane_model[2]]).transpose()
    R = from_two_vectors(p_init, p_goal)
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame().translate(x_y_z).rotate(R)

    pcd_list.extend([inlier_cloud, outlier_cloud, axis])
"""****************************************"""

pcd_raw = rgbd_image2pcd(img_rgb, img_depth, intrinsic)

pcd_list.append(pcd_raw)

o3d.visualization.draw_geometries(pcd_list,
                                  zoom=0.2,
                                  front=[0, 0, -1.0],
                                  lookat=[0, 0, 1.0],
                                  up=[0, -1.0, 0])
