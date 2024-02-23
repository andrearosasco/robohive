import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import numpy as np

from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg
from robotiq_85_msgs.msg import GripperStat

from rclpy.task import Future

class Robotiq(Node):

    max_width = 0.85
    gripper_state = None

    def __init__(self, **kwargs):
        return
        super().__init__('robotiq_action_client')
        self._action_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')

        self.create_subscription(GripperStat, "/gripper/stat", self._state_topic_callback, 1)

    def apply_commands(self, width:float, speed:float=0.1, force:float=0.1):
        print(width)
        goal_msg = GripperCommand.Goal(command=GripperCommandMsg(position=width, max_effort=5.0))
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self, self._send_goal_future)

    def get_sensors(self):
        self.gripper_state = Future()
        rclpy.spin_until_future_complete(self, self.gripper_state)
        return np.array([self.gripper_state.result().position])

    def connect(self):
        self._action_client.wait_for_server()

    def close(self):
        self.reset()
        rclpy.shutdown()

    def okay(self):
        return True

    def reset(self, width=0.1, **kwargs):
        self.apply_commands(width=0.1)

    def _state_topic_callback(self, msg):
        self.gripper_state.set_result(msg)
        self.gripper_state.done()