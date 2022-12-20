import numpy as np

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from franka_action_interface.action import Reach

from ReachSever import ReachSever


class ReachActionServer(Node):
    """
    The ROS2 node as an action server to provide an interface between physical manipulator and
    virtual reinforcement learning environment.
    """
    def __init__(self):
        super().__init__('reach_action_server')
        self._action_server = ActionServer(
            self,
            Reach,
            'Reach',
            self.execute_callback)
        self.sever = ReachSever()
        self.motion_sever = self.sever.motion_generate(5)
        next(self.motion_sever)

        self.get_logger().info("The sever has been initialized!")

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        goal_handle.succeed()

        # print(np.array(goal_handle.request.target_pose))
        self.sever.set_goal(np.array(goal_handle.request.target_relative_pose))

        waypoints = next(self.motion_sever)
        waypoints = waypoints.flatten().tolist()

        result = Reach.Result()
        result.waypoints = waypoints
        return result


def main(args=None):
    rclpy.init(args=args)

    reach_action_server = ReachActionServer()

    rclpy.spin(reach_action_server)


if __name__ == '__main__':
    main()
