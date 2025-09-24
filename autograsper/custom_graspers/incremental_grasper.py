import sys
import os

autograsper_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if autograsper_path not in sys.path:
    sys.path.append(autograsper_path)
    
from grasper import AutograsperBase, RobotActivity
from library.utils import OrderType
import time
import logging
import math
import pygetwindow as gw
from pynput import keyboard


# logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# listener for early termination when the active window is VS Code
def on_release(key):
    global quit

    active_window = gw.getActiveWindow()
    
    if active_window and "Visual Studio Code" in active_window.title:
        if key == keyboard.Key.esc:# and key == keyboard.KeyCode.from_char("q"):
            print("Early termination of the program")
            quit = True
            return False # stop listener


class IncrementalGrasperXZ(AutograsperBase):
    def __init__(self, config):
        super().__init__(config)

        # choose dimensions to explore and steps
        self.dimensions_to_explore = ["x", "z"]
        self.step = 0.01
        self.num_samples = 0
        self.max_num_samples = {"x": 100,"z": 100}
        self.time_between_orders = 0.05  # seconds

        self.robot_initial_pos = {"x": 0.0, "y": 0.5, "z": 0.0, "r": 0}
        self.robot_pos = self.robot_initial_pos.copy()

    def perform_task(self):

        for z in range(self.max_num_samples["z"]):

            # record initial state or after reset previous dimension
            self.record_current_state()
            self.robot_state, _ = self.robot.get_state()
            logger.info(f"Current state: (x={self.robot_pos['x']:.3f}, y={self.robot_pos['y']:.3f}, z={self.robot_pos['z']:.3f}, r={self.robot_pos['r']})")
            self.num_samples += 1

            for x in range(self.max_num_samples["x"]):
                
                # move x
                self.robot_pos["x"] += self.step
                self.queue_orders(
                    [
                        (OrderType.MOVE_XY, [self.robot_pos["x"], self.robot_pos["y"]])#self.robot_state["y_norm"]]),
                    ],
                    time_between_orders=self.time_between_orders,
                )
                # sleep() already inside queue_orders()
                # record_state() already inside queue_orders()
                self.robot_state, _ = self.robot.get_state()
                logger.info(f"Current state: (x={self.robot_pos['x']:.3f}, y={self.robot_pos['y']:.3f}, z={self.robot_pos['z']:.3f}, r={self.robot_pos['r']})")
                self.num_samples += 1                

                if quit:
                    break

            if self.num_samples % 100 == 0:
                    logger.info(f"Collected {self.num_samples} samples")

            # reset x
            self.robot_pos["x"] = self.robot_initial_pos["x"]
            # move z
            self.robot_pos["z"] += self.step
            self.queue_orders(
                [
                    (OrderType.MOVE_XY, [self.robot_pos["x"], self.robot_pos["y"]]),
                    (OrderType.MOVE_Z, [self.robot_pos["z"]])
                ],
                time_between_orders=self.time_between_orders,
            )

            if quit:
                break


        if not quit:
            listener.stop()
        logger.info(f"Recording session finished: {self.num_samples} samples collected")

        # comment or remove if you want multiple experiments to run
        self.state = RobotActivity.FINISHED  # stop data recording
        time.sleep(2)
        os._exit(1)

    def startup(self):
        # This method will execute at the beginning of every experiment.
        # During this phase, data will not be recorded or shown in real time.

        # listen for quit command
        global quit
        global listener
        quit = False
        listener = keyboard.Listener(on_release=on_release)
        listener.start()

        logger.info("Performing startup tasks...")

        # set initial robot position
        self.queue_orders(
            [
                (OrderType.MOVE_XY, [self.robot_initial_pos["x"], self.robot_initial_pos["y"]]),
                (OrderType.ROTATE, [self.robot_initial_pos["r"]]),
                (OrderType.MOVE_Z, [self.robot_initial_pos["z"]]),
                (OrderType.GRIPPER_OPEN, []),
            ],
            time_between_orders=self.time_between_orders,
        )
        time.sleep(2)  # wait for the robot to reach the position
        self.robot_state, _ = self.robot.get_state()
        logger.info(f"Initial state: {self.robot_state}")


    def reset_task(self):
        # replace with your own resetting if needed
        return super().reset_task()
    

