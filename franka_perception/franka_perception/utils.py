import numpy as np
from scipy.spatial.transform import Rotation as R


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


if __name__ == "__main__":
    p_init = np.array([0, 0, 1]).transpose()
    p_goal = np.array([1, 0, 0]).transpose()
    R = from_two_vectors(p_init, p_goal)
    print(np.matmul(R, p_init))
