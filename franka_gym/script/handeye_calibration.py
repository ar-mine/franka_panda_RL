# coding=utf-8
# copied by ysh in 2021/12/08
"""
用于相机标定和相机的手眼标定
A2^{-1}*A1*X=X*B2*B1^{−1}
"""

import os.path
import cv2
import numpy as np
np.set_printoptions(precision=8, suppress=True)
import glob

path = os.path.dirname(__file__)

# 角点的个数以及棋盘格间距
XX = 7
YY = 5
L = 0.008

# 设置寻找亚像素角点的参数，采用的停止准则是最大循环次数30和最大误差容限0.001
criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)

# 获取标定板角点的位置
objp = np.zeros((XX * YY, 3), np.float32)
objp[:, :2] = np.mgrid[0:XX, 0:YY].T.reshape(-1, 2)     # 将世界坐标系建在标定板上，所有点的Z坐标全部为0，所以只需要赋值x和y
objp = L*objp

obj_points = []     # 存储3D点
img_points = []     # 存储2D点

images = glob.glob(f'{path}/figure/*.jpg')
i = 0
for fname in images:
    img = cv2.imread(fname)
    # cv2.imshow('img',img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    size = gray.shape[::-1]
    ret, corners = cv2.findChessboardCorners(gray, (XX, YY), None)
    print(ret)

    if ret:

        obj_points.append(objp)

        corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 在原角点的基础上寻找亚像素角点
        #print(corners2)
        if [corners2]:
            img_points.append(corners2)
        else:
            img_points.append(corners)

        cv2.drawChessboardCorners(img, (XX, YY), corners, ret)
        # 红色为第一个点，蓝色为最后一个点，先X轴再Y轴
        cv2.imwrite(f'{path}/figure_save/{i}.jpg', img)
        i = i+1
        # cv2.imshow('img', img)
        # cv2.waitKey(2000)

N = len(img_points)
print(f'图像个数：{N}')
# cv2.destroyAllWindows()

# 标定,得到图案在相机坐标系下的位姿
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, size, None, None)

# print("ret:", ret)
print("内参矩阵:\n", mtx) # 内参数矩阵
print("畸变系数:\n", dist)  # 畸变系数   distortion cofficients = (k_1,k_2,p_1,p_2,k_3)
# print("rvecs:\n", rvecs)  # 旋转向量  # 外参数
# print("tvecs:\n", tvecs ) # 平移向量  # 外参数

print("-----------------------------------------------------")

# img2 = cv2.imread(f'{path}/figure/*.jpg')
i = 0
for fname in images:
    figure = cv2.imread(fname)
    h,  w = figure.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),0,(w,h)) # 自由比例参数
    dst = cv2.undistort(figure, mtx, dist, None, newcameramtx)
    cv2.imwrite(f'{path}/figure_undist/{i}.jpg', dst)
    i = i + 1

# 机器人末端在基座标系下的位姿
tool_pose = np.loadtxt(f'{path}/RobotToolPose.csv',delimiter=',')
R_tool = []
t_tool = []
for i in range(int(N)):
    R_tool.append(tool_pose[0:3,4*i:4*i+3])
    t_tool.append(tool_pose[0:3,4*i+3])

R, t = cv2.calibrateHandEye(R_tool, t_tool, rvecs, tvecs, cv2.CALIB_HAND_EYE_TSAI)
T_tool_camera = np.hstack((R, t))
T_tool_camera = np.vstack((T_tool_camera, np.array([0,0,0,1])))
print(f'相机在机器人末端坐标系的位姿：\n{T_tool_camera}')

with open(f'{path}/camera.txt', 'w') as f:
    f.write(f'{mtx}\n')
    f.write(f'{dist}\n')
    f.write(f'{T_tool_camera}')