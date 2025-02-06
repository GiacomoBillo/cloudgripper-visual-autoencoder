from grasper import AutograsperBase, RobotActivity
from library.utils import OrderType
import time
import numpy as np
import sys
import os
import cv2


autograsper_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if autograsper_path not in sys.path:
    sys.path.append(autograsper_path)


class GreedyGrasper(AutograsperBase):
    def __init__(self, config, max_num_samples, resolution_grid = 0.1, real_time_feed=True):
        super().__init__(config)

        # self.start_position
        # self.end_position

        self.num_samples = 0
        self.max_num_samples = max_num_samples
        self.resolution_grid = resolution_grid

        self.grid = Grid(shape=(10,10,10))

        self.real_time_feed = real_time_feed


    def perform_task(self):

        # while self.num_samples < self.max_num_samples:
        #     cell_x = self.grid.greedy_update(dim="x")
        #     new_x = np.random.rand() * self.resolution_grid + cell_x
        #     self.robot.move_xy(new_x, self.robot_pos["y_norm"])
        #     self.robot.get_all_states()
        #     time.sleep(self.time_between_orders)

        current_x = self.robot_state["x_norm"]
        current_y = self.robot_state["y_norm"]
        # current_z = self.robot_state["z_norm"]
        # current_rotation = self.robot_state["rotation"]
        # current_angle = self.robot_state["claw_norm"]
        
        while self.num_samples < self.max_num_samples:
            for dim in ["x", "y", "z"]:
                
                # get new coordinate along dimension dim
                cell = self.grid.greedy_update(dim=dim)
                new_coordinate = np.random.uniform(cell*self.resolution_grid, (cell+1)*self.resolution_grid)
                new_coordinate = self.resolution_grid * (np.random.rand() + cell)
                print(f"Cell: {cell}; New coordinate {dim}: {new_coordinate}")

                # apply move and wait
                if dim == "x":
                    #self.robot.move_xy(new_coordinate, self.robot_pos["y_norm"])
                    self.robot.move_xy(new_coordinate, current_y)
                    #self.robot.move_xy(np.random.rand(), np.random.rand())
                    #self.robot.move_xy(0.5, 0.5)
                    current_x = new_coordinate
                elif dim == "y":
                    # print("update y", self.robot_pos["x_norm"], self.robot_pos["y_norm"], new_coordinate)
                    #self.robot.move_xy(self.robot_pos["x_norm"], new_coordinate)
                    self.robot.move_xy(current_x, new_coordinate)
                    current_y = new_coordinate
                elif dim == "z":
                    self.robot.move_z(new_coordinate)
                    # current_z = new_coordinate
                time.sleep(self.time_between_orders)
                
                # get states
                #image_top, image_base, self.robot_pos, time_state = self.robot.get_all_states()
                self.record_current_state()
                self.robot_pos, _ = self.robot.get_state()
                self.num_samples += 1
                # time.sleep(self.time_between_orders)

                # save images

                # display images in real time
                # if self.real_time_feed:
                #     resized_images = [cv2.resize(image, (500, 500)) for image in [image_base, image_top]]
                #     concatenated_image = np.concatenate(resized_images, axis=1)
                #     cv2.imshow("Robot images", concatenated_image)

        # comment or remove if you want multiple experiments to run
        print("Greedy collection finished")
        self.state = RobotActivity.FINISHED  # stop data recording


    def startup(self):
        # This method will execute at the beginning of every experiment.
        # During this phase, data will not be recorded.

        print("performing startup tasks...")

        # if self.robot_pos == :
        #     # restore last position
        #     self.robot_pos = last
        #     self.robot.move_xy(self.robot_pos["x_norm"], self.robot_pos["y_norm"])
        #     self.robot.move_z(self.robot)
        # else:

        self.queue_orders(
            [
                (OrderType.MOVE_XY, [0, 0]),
                (OrderType.ROTATE, [0]),
                (OrderType.MOVE_Z, [0]),
                (OrderType.GRIPPER_OPEN, []),
                (OrderType.ROTATE, [0]),
            ],
            time_between_orders=self.time_between_orders  # set in config.ini file
        )

        self.robot_state, _ = self.robot.get_state()
        print(f"Initial state: {self.robot_state}")

    
    # def reset_task(self):
    #     with open('person.json', 'w') as file:
    #         json.dump(person_instance.to_dict(), file)


class Grid():
    def __init__(self, shape):
        self.grid = np.zeros(shape)
        self.pos = {"x":0, "y":0, "z":0}


    def greedy_update(self, dim):
        """
        Greedy choice of the cell along dim less explored
        """
        if dim == "x":
            cells_along_dim = self.grid[:, self.pos["y"], self.pos["z"]]
        elif dim == "y":
            cells_along_dim = self.grid[self.pos["x"], :, self.pos["z"]]
        elif dim == "z":
            cells_along_dim = self.grid[self.pos["x"], self.pos["y"], :]
        else:
            raise ValueError(f"Invalid dimension: {dim}")

        min_val = np.min(cells_along_dim)
        min_indeces = np.where(cells_along_dim == min_val)[0]

        new_coordinate = np.random.choice(min_indeces)
        
        self.pos[dim] = new_coordinate
        self.grid[self.pos["x"], self.pos["y"], self.pos["z"]] += 1

        return new_coordinate
    
