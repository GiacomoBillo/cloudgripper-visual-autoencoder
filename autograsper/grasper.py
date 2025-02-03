from abc import ABC, abstractmethod
import os
import sys
import time
from enum import Enum
from typing import List, Tuple
import ast

import numpy as np
from dotenv import load_dotenv

# Ensure the project root is in the system path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if project_root not in sys.path:
    sys.path.append(project_root)

# Import project-specific modules
from client.cloudgripper_client import GripperRobot
from library.utils import OrderType, queue_orders, clear_center, get_undistorted_bottom_image, execute_order

# Load environment variables
load_dotenv()


class RobotActivity(Enum):
    ACTIVE = 1
    RESETTING = 2
    FINISHED = 3
    STARTUP = 4


class AutograsperBase(ABC):
    def __init__(
        self,
        config,
        output_dir: str = "",
    ):
        self.token = os.getenv("CLOUDGRIPPER_TOKEN")
        if not self.token:
            raise ValueError("CLOUDGRIPPER_TOKEN environment variable not set")

        self.output_dir = output_dir
        self.start_time = time.time()
        self.failed = False

        self.state = RobotActivity.STARTUP
        self.start_flag = False
        
        self.request_state_record = False


        # fudge time to ensure frames at the start/finish of task/resetting
        self.task_time_margin = 2

        try:
            camera_cfg = config["camera"]
            self.camera_matrix = np.array(ast.literal_eval(camera_cfg["m"]))
            self.distortion_coeffs = np.array(ast.literal_eval(camera_cfg["d"]))
            self.record_only_after_action = bool(ast.literal_eval(camera_cfg["record_only_after_action"]))

            experiment_cfg = config["experiment"]
            self.robot_idx = ast.literal_eval(experiment_cfg["robot_idx"])
            self.time_between_orders = ast.literal_eval(experiment_cfg["time_between_orders"])

        except Exception as e:
            raise ValueError("Grasper config.ini ERROR: ", e) from e

        self.robot = self.initialize_robot(self.robot_idx, self.token)

        self.bottom_image = get_undistorted_bottom_image(
            self.robot, self.camera_matrix, self.distortion_coeffs
        )

    @staticmethod
    def initialize_robot(robot_idx: int, token: str) -> GripperRobot:
        try:
            return GripperRobot(robot_idx, token)
        except Exception as e:
            raise ValueError("Invalid robot ID or token: ", e) from e
    
    def record_current_state(self):
        self.request_state_record = True
        while self.request_state_record:
            time.sleep(0.05)

    def wait_for_start_signal(self):
        # TODO this solution can probably be improved
        while not self.start_flag:
            time.sleep(0.05)


    def run_grasping(self):
        while self.state != RobotActivity.FINISHED:
            self.startup()
            self.state = RobotActivity.ACTIVE

            self.wait_for_start_signal()
            self.start_flag = False

            try:
                self.perform_task()
            except Exception as e:
                print(f"Unexpected error during perform_task: {e}")
                self.failed = True
                raise

            time.sleep(self.task_time_margin) 
            self.state = RobotActivity.RESETTING
            time.sleep(self.task_time_margin)

            if self.failed:
                print("Experiment failed, recovering")
                self.recover_after_fail()
                self.failed = False
            else:
                self.reset_task()

            self.state = RobotActivity.STARTUP

    def recover_after_fail(self):
        pass

    @abstractmethod
    def perform_task(self):
        while True:
            print("GRASPER: No task defined. Override `perform_task` function to perform robot actions.")
            time.sleep(30)

    def reset_task(self):
        pass

    def startup(self):
        pass


    def queue_orders(
        self,
        order_list: List[Tuple[OrderType, List[float]]],
        time_between_orders: float,
        output_dir: str = "",
        reverse_xy: bool = False,
    ):
        """
        Queue a list of orders for the robot to execute sequentially and save state after each order.

        :param robot: The robot to execute the orders
        :param order_list: A list of tuples containing OrderType and the associated values
        :param time_between_orders: Time to wait between executing orders
        :param output_dir: Directory to save state data
        :param start_time: The start time of the autograsper process
        """
        for order in order_list:
            execute_order(self.robot, order, output_dir, reverse_xy)
            time.sleep(time_between_orders)
            if self.record_only_after_action and (self.state is RobotActivity.ACTIVE or self.state is RobotActivity.RESETTING):
                self.record_current_state()

