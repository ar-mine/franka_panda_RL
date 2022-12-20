import threading
import time
import copy

import rclpy
import std_msgs.msg
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from franka_interface.action import CartPoseSet
from franka_interface.srv import StackPick
from franka_perception.ros_utils import tf_msg2pose


# def stack_pick_callback(self, request, response):
#     idx = request.idx
#     bbox = self.bbox
#     len_bbox = len(bbox)
#     if idx >= len_bbox:
#         response.last_one = -1
#         response.target_pose = Pose()
#         self.get_logger().info("Wrong idx number!")
#
#     # Generate graph array
#     d_graph = np.zeros([len_bbox, len_bbox])
#     for i in range(len_bbox):
#         for j in range(i + 1, len_bbox):
#             ret = collision_check(bbox[i], bbox[j])
#             if ret == 1:
#                 d_graph[i, j] = 1
#             elif ret == 2:
#                 d_graph[j, i] = 1
#     print(d_graph)
#
#     response.last_one = 1
#     while not np.sum(d_graph[:, idx]) == 0:
#         response.last_one = 0
#         idx = np.where(d_graph[:, idx] == 1)[0][0]
#
#     response.target_pose = self.camera2world(bbox[idx])
#     return response

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


class MoveBoxNode(Node):
    def __init__(self):
        super().__init__('move_box_node')
        self.callback_group = ReentrantCallbackGroup()
        self.gripper_publisher = self.create_publisher(std_msgs.msg.Bool, "/moveit_cpp_node/gripper", qos_profile=3)

        self._action_client = ActionClient(self, CartPoseSet, "/moveit_cpp_node/move")
        self._target_subscription = self.create_subscription(Pose, "/target", callback=self.target_callback,
                                                             qos_profile=3, callback_group=self.callback_group)
        self._send_goal_future = None
        self._get_result_future = None
        self.finish = True

        self.pose = Pose()
        self.rate = self.create_rate(0.1)

        # Init service client for pose of target box
        self.pose_calc_client = self.create_client(StackPick, '/box_detector/pose_calc')
        while not self.pose_calc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        time.sleep(1)

    def send_goal_sync(self, cart_pose):
        goal_msg = CartPoseSet.Goal()
        goal_msg.target = cart_pose

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self.finish = False

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        while not self.finish:
            self.rate.sleep()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.finish = True
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        self.finish = True

    def target_callback(self, msg):
        self.pose = msg

    def get_target_pose(self, idx):
        """
        Use ros2 service to get the pose of the target pose.
        :param idx: the box to be picked
        :return:
        """
        req = StackPick.Request()
        req.idx = idx
        result = self.pose_calc_client.call(req)
        return result


def main(args=None):
    rclpy.init(args=args)

    move_box_node = MoveBoxNode()
    # gripper_publisher = ros_node.create_publisher(Bool, "/cart_position_controller_node/gripper", 3)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(move_box_node)
    t = threading.Thread(target=lambda: executor.spin(), name='LoopThread')
    t.start()

    # Wait to make system stable
    time.sleep(1)

    # pose = geometry_msgs.msg.Pose()
    # pose.position.x = 0.2
    # pose.position.y = -0.5
    # pose.position.z = 0.5
    # pose.orientation.x = 1.0
    # pose.orientation.y = 0.0
    # pose.orientation.z = 0.0
    # pose.orientation.w = 0.0
    msg = std_msgs.msg.Bool()
    msg.data = True
    pose = move_box_node.get_target_pose(1).target_pose
    if pose is not None:
        print(pose)
        pose.position.x -= 0.1
        move_box_node.send_goal_sync(pose)
        move_box_node.gripper_publisher.publish(msg)

    # idx_list = [0, 0, 0]
    # pose_calibration = [[0.0, -0.03], [0.02, 0.05], [0.05, 0.0]]
    #
    # for i, j in zip(idx_list, pose_calibration):
    #     result = ros_node.send_request(i)
    #     pose = result.target_pose
    #     last_one = result.last_one
    #     pose.position.x += j[0]
    #
    #     # Reach top of target, then target
    #     pose_copy = copy.deepcopy(pose)
    #     pose_copy.position.z += 0.05
    #     ros_node.send_goal_sync(pose_copy)
    #     ros_node.send_goal_sync(pose)
    #     # Suck box
    #     data = Bool()
    #     data.data = True
    #     gripper_publisher.publish(data)
    #     # Return to top of target
    #     ros_node.send_goal_sync(pose_copy)
    #
    #     # Target box, move to area
    #     if last_one:
    #         ros_node.move_with_position(0.4, -0.30, 0.25)
    #         ros_node.move_with_position(0.4, -0.30, 0.15)
    #     # else another area
    #     else:
    #         ros_node.move_with_position(0.4, 0.30, 0.25)
    #         ros_node.move_with_position(0.4, 0.30, 0.15+j[1])
    #     # Drop the box
    #     data.data = False
    #     gripper_publisher.publish(data)
    #
    #     # Return to the beginning
    #     ros_node.move_with_position(0.4, 0.0, 0.25)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
