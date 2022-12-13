import open3d as o3d
import numpy as np
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField


def o3d_pcd2numpy(o3d_pcd: o3d.geometry.PointCloud()):
    """
        Convert point cloud (open3D format) to numpy array (xyzrgb for ROS PointCloud2 format)
    """
    points = np.asarray(o3d_pcd.points)
    colors = np.asarray(o3d_pcd.colors)
    np_array = np.concatenate([points, colors], axis=1)
    return np_array


def rgbd_image2pcd(img_rgb, img_depth, intrinsic):
    """
    Convert color and depth image (numpy array format) to point cloud (open3D format)
    """
    color_raw = o3d.geometry.Image(img_rgb.astype(np.uint8))
    depth_raw = o3d.geometry.Image(img_depth.astype(np.uint16))
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    return pcd


def np_pcd2ros_msg(points, parent_frame, time):
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