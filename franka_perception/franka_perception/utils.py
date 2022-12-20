import numpy as np
from scipy.spatial.transform import Rotation as R

from franka_perception.ros_utils import pose2numpy


def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])


def from_two_vectors(p_init, p_goal):
    v = np.cross(p_init, p_goal)
    s = np.linalg.norm(v)
    c = np.dot(p_init, p_goal)
    R = np.eye(3) + skew(v) * s + np.matmul(skew(v), skew(v)) * (1 - c)
    return R


def matrix2quat(m):
    return R.from_matrix(m).as_quat()


def pose_multiply(pose_base, pose_relative):
    pose_base = pose2numpy(pose_base)
    pose_relative = pose2numpy(pose_relative)

    matrix_base = R.from_quat(pose_base[3:]).as_matrix()
    matrix_relative = R.from_quat(pose_relative[3:]).as_matrix()

    t_base = np.eye(4)
    t_relative = np.eye(4)
    t_base[:3, :3] = matrix_base
    t_relative[:3, :3] = matrix_relative
    t_base[:3, 3] = pose_base[:3]
    t_relative[:3, 3] = pose_relative[:3]

    t = np.matmul(t_base, t_relative)
    return np.concatenate([t[:3, 3], R.from_matrix(t[:3, :3]).as_quat()])


if __name__ == "__main__":
    t_base = np.array([-0.40459, -0.56721, 0.24523, 0.51955, -0.48197, 0.5129, -0.48447])
    t_relative = np.array([-0.08, 0.015, 0.78, -0.001, -0.004, 0, 0.999])
    print(pose_multiply(t_base, t_relative))
