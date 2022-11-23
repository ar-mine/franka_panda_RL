import threading
import time
import copy

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

from franka_interface.action import CartPoseSet
from franka_interface.srv import StackPick


class MoveBoxNode(Node):
    def __init__(self):
        super().__init__('move_box_node')
        self._action_client = ActionClient(self, CartPoseSet, 'cart_pose_set')
        self._target_subscription = self.create_subscription(Pose, "/target",
                                                             callback=self.target_callback, qos_profile=3)
        self._send_goal_future = None
        self._get_result_future = None
        self.finish = True

        self.cli = self.create_client(StackPick, 'stack_pick')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = StackPick.Request()

        self.pose = Pose()

        self.rate = self.create_rate(0.5)

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

    def send_request(self, idx):
        self.req.idx = idx
        result = self.cli.call(self.req)
        return result

    def move_with_position(self, x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        self.send_goal_sync(pose)


def move_box_idx(idx):
    pass


def main(args=None):
    rclpy.init(args=args)

    ros_node = MoveBoxNode()
    gripper_publisher = ros_node.create_publisher(Bool, "/cart_position_controller_node/gripper", 3)
    t = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    t.start()

    time.sleep(1)

    idx_list = [0, 0, 0]
    pose_calibration = [[0.0, -0.03], [0.02, 0.05], [0.05, 0.0]]

    for i, j in zip(idx_list, pose_calibration):
        result = ros_node.send_request(i)
        pose = result.target_pose
        last_one = result.last_one
        pose.position.x += j[0]

        # Reach top of target, then target
        pose_copy = copy.deepcopy(pose)
        pose_copy.position.z += 0.05
        ros_node.send_goal_sync(pose_copy)
        ros_node.send_goal_sync(pose)
        # Suck box
        data = Bool()
        data.data = True
        gripper_publisher.publish(data)
        # Return to top of target
        ros_node.send_goal_sync(pose_copy)

        # Target box, move to area
        if last_one:
            ros_node.move_with_position(0.4, -0.30, 0.25)
            ros_node.move_with_position(0.4, -0.30, 0.15)
        # else another area
        else:
            ros_node.move_with_position(0.4, 0.30, 0.25)
            ros_node.move_with_position(0.4, 0.30, 0.15+j[1])
        # Drop the box
        data.data = False
        gripper_publisher.publish(data)

        # Return to the beginning
        ros_node.move_with_position(0.4, 0.0, 0.25)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
