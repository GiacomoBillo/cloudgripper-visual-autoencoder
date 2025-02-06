from grasper import AutograsperBase, RobotActivity
from library.utils import OrderType, parse_config
import time
import numpy as np
import sys
import os
import cv2
import json
import ast


autograsper_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if autograsper_path not in sys.path:
    sys.path.append(autograsper_path)


class GreedyGrasper(AutograsperBase):
    def __init__(self, config, max_num_samples, resolution_grid=0.1):
        super().__init__(config)

        self.num_samples = 0
        self.max_num_samples = max_num_samples
        self.resolution_grid = resolution_grid

        experiment_name = parse_config(config)["experiment"].get("name").strip('"')
        self.restore_grasper_file = os.path.join(autograsper_path, "recorded_data", experiment_name, "restore_grasper.json")
        # if the same experiment was already started
        if os.path.exists(self.restore_grasper_file):
            # restore grid
            with open(self.restore_grasper_file, "r") as file:
                encoded = ast.literal_eval(file.read())
                self.grid = Grid.decode_grid(encoded)
                # self.grid = json.load(Grid.decode_grid(encoded))
                # self.grid = json.load(file, object_hook=lambda dct: Grid.decode_grid(dct))
            print("Grasper restored")
        else:
            # initialize grid
            # self.grid = Grid(shape=(10,10,10))  # 3D
            self.grid = Grid(shape=(10,10,10,10,10))  # 5D



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
            #for dim in ["x", "y", "z"]:
            for dim in ["x", "y", "z", "r", "g"]:

                # get new coordinate along dimension dim
                cell = self.grid.greedy_update(dim=dim)
                # new_coordinate = np.random.uniform(cell*self.resolution_grid, (cell+1)*self.resolution_grid)
                new_coordinate = self.resolution_grid * (np.random.rand() + cell)
                print(f"Cell: {cell}; New coordinate {dim}: {new_coordinate}")

                # apply move and wait
                if dim == "x":
                    self.robot.move_xy(new_coordinate, current_y)
                    current_x = new_coordinate
                elif dim == "y":
                    self.robot.move_xy(current_x, new_coordinate)
                    current_y = new_coordinate
                elif dim == "z":
                    self.robot.move_z(new_coordinate)
                elif dim == "r":
                    # scale the coordinate [0,1) -> [0,180)
                    self.robot.rotate(int(new_coordinate * 180))
                elif dim == "g":
                    self.robot.move_gripper(new_coordinate)
                time.sleep(self.time_between_orders)
                
                # get states
                #image_top, image_base, self.robot_pos, time_state = self.robot.get_all_states()
                self.record_current_state()
                self.robot_pos, _ = self.robot.get_state()
                self.num_samples += 1
                # time.sleep(self.time_between_orders)

                if self.num_samples >= self.max_num_samples:
                    break

                # save images

                # display images in real time
                # if self.real_time_feed:
                #     resized_images = [cv2.resize(image, (500, 500)) for image in [image_base, image_top]]
                #     concatenated_image = np.concatenate(resized_images, axis=1)
                #     cv2.imshow("Robot images", concatenated_image)

        with open(self.restore_grasper_file, 'w') as file:
            json.dump(self.grid.encode_grid(), file)
        # comment or remove if you want multiple experiments to run
        print("Greedy collection finished")
        self.state = RobotActivity.FINISHED  # stop data recording


    def startup(self):
        # This method will execute at the beginning of every experiment.
        # During this phase, data will not be recorded.

        print("performing startup tasks...")

        # position 0
        self.queue_orders(
            [
                (OrderType.MOVE_XY, [0, 0]),
                (OrderType.MOVE_Z, [0]),
                (OrderType.ROTATE, [0]),
                (OrderType.GRIPPER_OPEN, []),
            ],
            time_between_orders=self.time_between_orders  # set in config.ini file
        )
        
        # random position
        # pos = {}
        # # greedy choice of position (random if the grid is new)
        # for dim in ["x", "y", "z", "r", "g"]:
        #     cell = self.grid.greedy_update(dim=dim)
        #     pos[dim] = self.resolution_grid * (np.random.rand() + cell)
        # self.queue_orders(
        #     [
        #         (OrderType.MOVE_XY, [pos["x"], pos["y"]]),
        #         (OrderType.MOVE_Z, [pos["z"]]),
        #         (OrderType.ROTATE, [pos["r"]]),
        #         #(OrderType.GRIPPER_OPEN, []),
        #     ],
        #     time_between_orders=self.time_between_orders  # set in config.ini file
        # )
        # self.robot.move_gripper(pos["g"])
        # time.sleep(self.time_between_orders)

        self.robot_state, _ = self.robot.get_state()
        #self.grid.set_position(self.robot_state)  # use only when x and y are normalized
        print(f"Initial state: {self.robot_state}")

    
    # def reset_task(self):
    #     with open(self.restore_grasper_file, 'w') as file:
    #         json.dump(self.grid, file)


class Grid():
    def __init__(self, shape):
        self.shape = shape
        self.grid = np.zeros(shape)
        #self.pos = {"x":0, "y":0, "z":0}
        self.pos = {"x":0, "y":0, "z":0, "r":0, "g":0}


    def greedy_update(self, dim):
        """
        Greedy choice of the cell along dim less explored
        """
        # if dim == "x":
        #     cells_along_dim = self.grid[:, self.pos["y"], self.pos["z"]]
        # elif dim == "y":
        #     cells_along_dim = self.grid[self.pos["x"], :, self.pos["z"]]
        # elif dim == "z":
        #     cells_along_dim = self.grid[self.pos["x"], self.pos["y"], :]
        if dim == "x":
            cells_along_dim = self.grid[:, self.pos["y"], self.pos["z"], self.pos["r"], self.pos["g"]]
        elif dim == "y":
            cells_along_dim = self.grid[self.pos["x"], :, self.pos["z"], self.pos["r"], self.pos["g"]]
        elif dim == "z":
            cells_along_dim = self.grid[self.pos["x"], self.pos["y"], :, self.pos["r"], self.pos["g"]]
        elif dim == "r":
            cells_along_dim = self.grid[self.pos["x"], self.pos["y"], self.pos["z"], :, self.pos["g"]]
        elif dim == "g":
            cells_along_dim = self.grid[self.pos["x"], self.pos["y"], self.pos["z"], self.pos["r"], :]
        else:
            raise ValueError(f"Invalid dimension: {dim}")


        min_val = np.min(cells_along_dim)
        min_indeces = np.where(cells_along_dim == min_val)[0]

        new_coordinate = np.random.choice(min_indeces)
        
        self.pos[dim] = new_coordinate
        self.grid[self.pos["x"], self.pos["y"], self.pos["z"], self.pos["r"], self.pos["g"]] += 1

        return new_coordinate

    def get_tot_samples(self):
        return np.sum(self.grid)
    
    def set_position(self, robot_state: dict):
        self.pos["x"] = robot_state["x_norm"] * self.shape[0]
        self.pos["y"] = robot_state["y_norm"] * self.shape[1]
        self.pos["z"] = robot_state["z_norm"] * self.shape[2]
        self.pos["r"] = robot_state["rotation"] * self.shape[3] / 180
        self.pos["g"] = robot_state["claw_norm"] * self.shape[4]
    
    def encode_grid(self):
        return {
            "shape": tuple(int(dim) for dim in self.shape),
            "grid": self.grid.flatten().astype(int).tolist(),
            "pos": {key: int(val) for key, val in self.pos.items()}
            # "shape": ",".join(self.shape),
            # "grid": ",".join(self.grid.flatten().tolist()),
            # "pos": str(self.pos)
        }
    
    @classmethod
    def decode_grid(cls, encoded_obj):
        print(encoded_obj)

        if not all(key in encoded_obj for key in ["shape", "grid", "pos"]):
            raise ValueError("Missing required fields in the JSON file")
        
        obj = Grid(encoded_obj["shape"])
        obj.grid =  np.array(encoded_obj["grid"]).reshape(obj.shape)
        obj.pos = encoded_obj["pos"]  
        # obj = Grid(encoded_obj["shape"].split(","))
        # obj.grid =  np.array(encoded_obj["grid"].split(",")).reshape(obj.shape)
        # obj.pos = ast.literal_eval(encoded_obj["pos"])  
        return obj

