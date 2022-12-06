import numpy as np
import cv2 as cv
import open3d as o3d
import matplotlib.pyplot as plt
from franka_perception.yolov5.detector import YoloV5

intrinsic = o3d.camera.PinholeCameraIntrinsic(
        640, 480, 616.0755615234375, 616.6409912109375, 335.7129211425781, 234.61709594726562)

K = np.array([[616.0755615234375, 0.0, 335.7129211425781],
              [0.0, 616.6409912109375, 234.61709594726562],
              [0.0, 0.0, 1.0]])
detector = YoloV5()

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
    result_rgb_o3d = o3d.geometry.Image(result_rgb.astype(np.uint8))
    result_depth_o3d = o3d.geometry.Image(result_depth)
    result_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        result_rgb_o3d, result_depth_o3d)
    result_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(result_rgbd, intrinsic)

    plane_model, inliers = result_pcd.segment_plane(distance_threshold=0.01,
                                                     ransac_n=3,
                                                     num_iterations=1000)

    inlier_cloud = result_pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = result_pcd.select_by_index(inliers, invert=True)

    pcd_list.extend([inlier_cloud, outlier_cloud])
"""****************************************"""

color_raw = o3d.geometry.Image(img_rgb.astype(np.uint8))
depth_raw = o3d.geometry.Image(img_depth)
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)
pcd_raw = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)


pcd_list.append(pcd_raw)

o3d.visualization.draw_geometries(pcd_list,
                                  zoom=0.2,
                                  front=[0, 0, -1.0],
                                  lookat=[0, 0, 1.0],
                                  up=[0, -1.0, 0])
