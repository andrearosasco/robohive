import time
import numpy as np

from panda_interface.srv import ApplyCommands, Connect, GetSensors, Close
import rclpy
from rclpy.node import Node
from panda_interface.msg import PandaCommand

from robohive.utils.min_jerk import generate_joint_space_min_jerk



class FrankaArm(Node):

    interfaces = {
        'apply_commands': ApplyCommands,
        'get_sensors': GetSensors,
        'connect': Connect,
        'close': Close
    }

    def __init__(self, **kwargs):
        super().__init__('panda_client')

        self.client_names = {}
        for name, type in FrankaArm.interfaces.items():
            client = self.create_client(type, name)

            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'service {name} not available, waiting again...')

            self.client_names[name] = client

    def connect(self):
        request = FrankaArm.interfaces['connect'].Request()
        self.future = self.client_names['connect'].call_async(request)

        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    def apply_commands(self, q_desired=None, kp=None, kd=None, gain=4.):
        request = FrankaArm.interfaces['apply_commands'].Request()

        request.command = PandaCommand(position=q_desired, gain=gain)
        self.future = self.client_names['apply_commands'].call_async(request)

        return

    def get_sensors(self):
        request = FrankaArm.interfaces['get_sensors'].Request()

        self.future = self.client_names['get_sensors'].call_async(request)
        rclpy.spin_until_future_complete(self, self.future)

        state = self.future.result().state

        return {'joint_pos': state.position, 'joint_vel': state.velocity}

    def close(self):
        self.reset()

        request = FrankaArm.interfaces['close'].Request()
        self.future = self.client_names['close'].call_async(request)

        rclpy.shutdown()

    def reset(self, reset_pos=None, time_to_go=5):
        home = np.array([0.0, 0.3, 0.0, -1.15, 0.0, 1.5, 0.0])

        # Use registered controller
        q_current = self.get_sensors()['joint_pos']
        # generate min jerk trajectory
        dt = 0.1
        waypoints =  generate_joint_space_min_jerk(start=q_current, goal=home, time_to_go=5, dt=dt)
        # reset using min_jerk traj
        for i in range(len(waypoints)):
            self.apply_commands(q_desired=waypoints[i]['position'], gain=10.)
            time.sleep(dt)

    def okay(self):
        return True
