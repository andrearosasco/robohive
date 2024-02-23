import time
import numpy as np

import rclpy
from rclpy.node import Node

from panda_interface.srv import ApplyCommandsGripper, ConnectGripper, GetSensorsGripper
from panda_interface.msg import PandaGripperCommand



class FrankaHand(Node):

    interfaces = {
        'apply_commands_gripper': ApplyCommandsGripper,
        'get_sensors_gripper': GetSensorsGripper,
        'connect_gripper': ConnectGripper,
    }

    def __init__(self, **kwargs):
        super().__init__('panda_gripper_client')

        self.client_names = {}
        for name, type in FrankaHand.interfaces.items():
            client = self.create_client(type, name)

            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'service {name} not available, waiting again...')

            self.client_names[name] = client

    def connect(self):
        request = FrankaHand.interfaces['connect_gripper'].Request()
        self.future = self.client_names['connect_gripper'].call_async(request)

        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    def apply_commands(self, width:float, speed:float=0.1, force:float=0.1):
        request = FrankaHand.interfaces['apply_commands_gripper'].Request()

        request.command = PandaGripperCommand(width=width)
        self.future = self.client_names['apply_commands_gripper'].call_async(request)
        # rclpy.spin_until_future_complete(self, self.future)

        return

    def get_sensors(self):
        request = FrankaHand.interfaces['get_sensors_gripper'].Request()

        self.future = self.client_names['get_sensors_gripper'].call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

        state = self.future.result().state

        return np.array([state.width])

    def close(self):
        self.reset()
        rclpy.shutdown()

    def reset(self, width=0.1, **kwargs):
        self.apply_commands(width=width)

    def okay(self):
        return True
