from typing import Union
import numpy as np

from std_msgs.msg import Header, Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from geometry_msgs.msg import TransformStamped, Pose
from sensor_msgs.msg import PointCloud2, PointField


def np2tf_msg(xyz_xyzw, time, frame_parent, frame_child):
    t = TransformStamped()

    # Read message content and assign it to
    # corresponding tf variables
    t.header.stamp = time
    t.header.frame_id = frame_parent
    t.child_frame_id = frame_child

    # Turtle only exists in 2D, thus we get x and y translation
    # coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = xyz_xyzw[0]
    t.transform.translation.y = xyz_xyzw[1]
    t.transform.translation.z = xyz_xyzw[2]

    # For the same reason, turtle can only rotate around one axis
    # and this why we set rotation in x and y to 0 and obtain
    # rotation in z axis from the message
    t.transform.rotation.x = xyz_xyzw[3]
    t.transform.rotation.y = xyz_xyzw[4]
    t.transform.rotation.z = xyz_xyzw[5]
    t.transform.rotation.w = xyz_xyzw[6]

    return t


def np2pose(xyz_xyzw):
    t = Pose()

    t.position.x = xyz_xyzw[0]
    t.position.y = xyz_xyzw[1]
    t.position.z = xyz_xyzw[2]

    t.orientation.x = xyz_xyzw[3]
    t.orientation.y = xyz_xyzw[4]
    t.orientation.z = xyz_xyzw[5]
    t.orientation.w = xyz_xyzw[6]

    return t


def np_pcd2ros_msg(points, time, parent_frame):
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

    header = Header(frame_id=parent_frame, stamp=time)

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


def np2multi_array(np_array: np.ndarray) -> Float32MultiArray:
    shape = np_array.shape
    dim = len(shape)
    stride = 1

    layout_msg = MultiArrayLayout()
    for i in reversed(range(dim)):
        dim_msg = MultiArrayDimension()
        stride *= shape[i]
        dim_msg.stride = stride
        dim_msg.size = shape[i]
        dim_msg.label = str(i)
        layout_msg.dim.append(dim_msg)
    layout_msg.dim.reverse()

    float_array = Float32MultiArray()
    float_array.data = np_array.astype(float).flatten().tolist()
    float_array.layout = layout_msg

    return float_array


def tf_msg2pose(tf_msg: TransformStamped) -> Pose:
    pose = Pose()
    pose.position.x = tf_msg.transform.translation.x
    pose.position.y = tf_msg.transform.translation.y
    pose.position.z = tf_msg.transform.translation.z
    pose.orientation.x = tf_msg.transform.rotation.x
    pose.orientation.y = tf_msg.transform.rotation.y
    pose.orientation.z = tf_msg.transform.rotation.z
    pose.orientation.w = tf_msg.transform.rotation.w
    return pose


def pose2numpy(pose: Union[Pose, TransformStamped, np.ndarray]) -> np.ndarray:
    if type(pose) is not np.ndarray:
        if type(pose) is Pose:
            pose = np.array([pose.position.x, pose.position.y, pose.position.z,
                             pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        elif type(pose) is TransformStamped:
            pose = np.array([pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z,
                             pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w])
        return pose
    else:
        return pose
