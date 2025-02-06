from grasper import AutograsperBase, RobotActivity
from library.utils import OrderType
import time
import numpy as np


class GreedyGrasper(AutograsperBase):
    def __init__(self, config, max_num_samples, resolution_grid = 0.1):
        super().__init__(config)

        self.start_position
        self.end_position

        self.num_samples = 0
        self.max_num_samples = max_num_samples
        self.resolution_grid = resolution_grid

        self.grid = Grid(shape=(10,10,10))


    def perform_task(self):

        # while self.num_samples < self.max_num_samples:
        #     cell_x = self.grid.greedy_update(dim="x")
        #     new_x = np.random.rand() * self.resolution_grid + cell_x
        #     self.robot.move_xy(new_x, self.robot_state["y_norm"])
        #     self.robot.get_all_states()
        #     time.sleep(self.time_between_orders)

        while self.num_samples < self.max_num_samples:
            for dim in ["x", "y", "z"]:
                # get new coordinate along dimension dim
                cell = self.grid.greedy_update(dim=dim)
                new_coordinate = np.random.rand() * self.resolution_grid + cell

                # apply move and wait
                if dim == "x":
                    self.robot.move_xy(new_coordinate, self.robot_state["y_norm"])
                elif dim == "y":
                    self.robot.move_xy(self.robot_state["x_norm"], new_coordinate)
                elif dim == "z":
                    self.robot.move_z(new_coordinate)
                time.sleep(self.time_between_orders)
                
                # get states
                # image_top, image_base, self.robot_state, time_state = self.robot.get_all_states()
                self.record_current_state()
                # time.sleep(self.time_between_orders)

                # save images

                # display images in real time
                # if real_time:


        # comment or remove if you want multiple experiments to run
        print("Greedy collection finished")
        self.state = RobotActivity.FINISHED  # stop data recording




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
        min_indeces = np.where(cells_along_dim == min_val)
        new_coordinate = np.random.choice(min_indeces)
        
        self.pos[dim] = new_coordinate
        self.grid[self.pos["x"], self.pos["y"], self.pos["z"]] += 1

        return new_coordinate