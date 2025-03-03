import sys
import os

autograsper_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if autograsper_path not in sys.path:
    sys.path.append(autograsper_path)

from grasper import AutograsperBase, RobotActivity
from library.utils import OrderType, parse_config
import time
import numpy as np
import cv2
import json
import ast
from pynput import keyboard
import copy
import pygetwindow as gw
import argparse


quit = False
listener = None

def on_release(key):
    global quit

    active_window = gw.getActiveWindow()
    
    if active_window and "Visual Studio Code" in active_window.title:
        if key == keyboard.Key.esc:# and key == keyboard.KeyCode.from_char("q"):
            print("Early termination of the program")
            quit = True
            return False # stop listener
    

def coordinate_to_idx(coordinate, dim):
    # normalize coordinate
    if coordinate<0:
        coordinate = 0
    elif coordinate>1:
        coordinate = 1

    idx = int(coordinate * dim)
    if idx >= dim: # case coordinate = 1
        idx = dim-1
    return idx


"""
Automatic grapser that explores the 
"""
class GreedyGrasper(AutograsperBase):
    def __init__(self, config, max_num_samples, resolution_grid=0.1):
        super().__init__(config)

        self.num_samples = 0
        self.max_num_samples = max_num_samples
        self.resolution_grid = resolution_grid

        experiment_name = parse_config(config)["experiment"].get("name").strip('"')
        self.restore_grasper_file = os.path.join(autograsper_path, "recorded_data", experiment_name, "restore_grasper.json")
        
        self.grid = None


    def perform_task(self):
        global quit

        current_x = self.robot_state["x_norm"]
        current_y = self.robot_state["y_norm"]
        
        while self.num_samples < self.max_num_samples and not quit:
            #for dim in ["x", "y", "z"]:
            for dim in ["x", "y", "z", "r", "g"]:

                # get new coordinate along dimension dim
                cell = self.grid.greedy_update(dim=dim)
                # new_coordinate = np.random.uniform(cell*self.resolution_grid, (cell+1)*self.resolution_grid)
                new_coordinate = self.resolution_grid * (np.random.rand() + cell)
                print(f"Dim: {dim}; Cell: {cell}; New coordinate {dim}: {new_coordinate}")

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
                    self.robot.rotate(round(new_coordinate * 180))
                elif dim == "g":
                    self.robot.move_gripper(new_coordinate)
                time.sleep(self.time_between_orders)
                
                # get states
                # image_top, image_base, self.robot_pos, time_state = self.robot.get_all_states()
                self.record_current_state()
                self.robot_pos, _ = self.robot.get_state()
                self.num_samples += 1
                # time.sleep(self.time_between_orders)

                if self.num_samples >= self.max_num_samples or quit:
                    break

                if self.num_samples%100 == 0:
                    print(f"Number of samples taken {self.num_samples}/{self.max_num_samples}")

                # save images

                # display images in real time
                # if self.real_time_feed:
                #     resized_images = [cv2.resize(image, (500, 500)) for image in [image_base, image_top]]
                #     concatenated_image = np.concatenate(resized_images, axis=1)
                #     cv2.imshow("Robot images", concatenated_image)

        if not quit:
            listener.stop()
        self.grid.write_on_file(self.restore_grasper_file)
        print(f"Greedy collection session finished: {self.num_samples}/{self.max_num_samples} samples"
              f"\nTotal number of samples on the distribution grid: {self.grid.get_tot_samples()}")
        
        # comment or remove if you want multiple experiments to run
        self.state = RobotActivity.FINISHED  # stop data recording


    def startup(self):
        # This method will execute at the beginning of every experiment.
        # During this phase, data will not be recorded.

        # listen for quit command
        global quit
        global listener
        quit = False
        listener = keyboard.Listener(on_release=on_release)
        listener.start()

        print("performing startup tasks...")

        # if the same experiment was already started
        if os.path.exists(self.restore_grasper_file):
            # restore grid
            with open(self.restore_grasper_file, "r") as file:
                encoded = ast.literal_eval(file.read())
                self.grid = Grid.decode_grid(encoded)
                # self.grid = json.load(Grid.decode_grid(encoded))
                # self.grid = json.load(file, object_hook=lambda dct: Grid.decode_grid(dct))
            print("Samples distribution grid restored, total number of samples:", self.grid.get_tot_samples())
        else:
            # initialize grid
            # self.grid = Grid(shape=(10,10,10))  # 3D
            self.grid = Grid(shape=(10,10,10,10,10))  # 5D

        # # position 0
        # self.queue_orders(
        #     [
        #         (OrderType.MOVE_XY, [0, 0]),
        #         (OrderType.MOVE_Z, [0]),
        #         (OrderType.ROTATE, [0]),
        #         (OrderType.GRIPPER_OPEN, []),
        #     ],
        #     time_between_orders=self.time_between_orders  # set in config.ini file
        # )
    
        # initialize position in the less sampled cell of the grid 
        start_pos = self.grid.less_sampled_cell()
        print(f"Initial position on grid: {start_pos}")

        # randomize coordinate in cell
        for key in start_pos:
            start_pos[key] = self.resolution_grid * (np.random.rand() + start_pos[key])
        
        self.queue_orders(
            [
                (OrderType.MOVE_XY, [start_pos["x"], start_pos["y"]]),
                (OrderType.MOVE_Z, [start_pos["z"]]),
                (OrderType.ROTATE, [round(start_pos["r"]*180)]),
            ],
            time_between_orders=self.time_between_orders  # set in config.ini file
        )
        self.robot.move_gripper(start_pos["g"])
        time.sleep(self.time_between_orders)

        self.robot_state, _ = self.robot.get_state()
        #self.grid.set_position(self.robot_state)  # use only when x and y are normalized
        print(f"Initial state: {self.robot_state}")

    
    # def reset_task(self): # does not work
    #     with open(self.restore_grasper_file, 'w') as file:
    #         json.dump(self.grid, file)


class Grid():
    def __init__(self, shape):
        self.shape = shape
        self.grid = np.zeros(shape)
        #self.pos = {"x":0, "y":0, "z":0}
        self.pos = {"x":0, "y":0, "z":0, "r":0, "g":0}

    def less_sampled_cell(self):
        less_sampled = np.unravel_index(np.argmin(self.grid), self.grid.shape)
        
        self.pos["x"] = less_sampled[0]
        self.pos["y"] = less_sampled[1]
        self.pos["z"] = less_sampled[2]
        self.pos["r"] = less_sampled[3]
        self.pos["g"] = less_sampled[4]

        return copy.deepcopy(self.pos)

    def greedy_update(self, dim):
        """
        Greedy choice of the cell along dim less explored
        """

        # 3D
        # if dim == "x":
        #     cells_along_dim = self.grid[:, self.pos["y"], self.pos["z"]]
        # elif dim == "y":
        #     cells_along_dim = self.grid[self.pos["x"], :, self.pos["z"]]
        # elif dim == "z":
        #     cells_along_dim = self.grid[self.pos["x"], self.pos["y"], :]

        # 5D
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

        # # if more than one cell is available, avoid the current cell
        # if len(min_indeces) > 1 and self.pos[dim] in min_indeces:
        #     min_indeces = np.delete(min_indeces, self.pos[dim])
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

    def store_sample(self, robot_state):
        """
        to store a sample in the distribution grid
        useful to store samples already taken but not stored on the grid
        """
        x = coordinate_to_idx(robot_state["x_norm"], self.shape[0])
        y = coordinate_to_idx(robot_state["y_norm"], self.shape[1])
        z = coordinate_to_idx(robot_state["z_norm"], self.shape[2])
        r = coordinate_to_idx(robot_state["rotation"]/180, self.shape[3])
        g = coordinate_to_idx(robot_state["claw_norm"], self.shape[4])
        print(x,y,z,r,g)
        print("Prev val =", self.grid[x,y,z,r,g])
        self.grid[x,y,z,r,g] += 1
        print("Post val =", self.grid[x,y,z,r,g])
    
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

        if not all(key in encoded_obj for key in ["shape", "grid", "pos"]):
            raise ValueError("Missing required fields in the JSON file")
        
        obj = Grid(encoded_obj["shape"])
        obj.grid =  np.array(encoded_obj["grid"]).reshape(obj.shape)
        obj.pos = encoded_obj["pos"]  
        # obj = Grid(encoded_obj["shape"].split(","))
        # obj.grid =  np.array(encoded_obj["grid"].split(",")).reshape(obj.shape)
        # obj.pos = ast.literal_eval(encoded_obj["pos"])  
        return obj
    
    def read_from_file():
        pass

    def write_on_file(self, filename):
        with open(filename, 'w') as file:
            json.dump(self.encode_grid(), file)

    def print_sample_positions(self):
        sampled_positions = np.argwhere(self.grid > 0)
        print("Sampled positions:")
        for pos in sampled_positions:
            pos = tuple(map(int, pos))
            count = int(self.grid[tuple(pos)])
            print(f"Position {tuple(pos)} -> Count: {count}")


def store_samples_on_grid(grid: Grid, session_num):
    """
    to store the samples of a recording session on the grid
    if an error stopped the program before saving the grid
    """
    print("Store samples session", args.store_session, "on the grid")
    state_file = os.path.join(autograsper_path, "recorded_data", experiment_name, session_num, "task", "states.json")

    # open state file
    if os.path.exists(state_file):
        with open(state_file, "r") as file:
            data = json.load(file)
    else :
        raise Exception(f"File {state_file} not found")
    
    num_samples = len(data)
    for i, state in enumerate(data):
        # store sample on grid
        grid.store_sample(state)
        # print(f"Sample {i+1}/{num_samples} stored on grid")
    print("New samples stored:", num_samples)



if __name__ == "__main__":

    config_file = "autograsper/config.ini"
    experiment_name = parse_config(config_file)["experiment"].get("name").strip('"')
    restore_grasper_file = os.path.join(autograsper_path, "recorded_data", experiment_name, "restore_grasper.json")
    
    # store state session on experiment grid
    if os.path.exists(restore_grasper_file):
        # restore grid
        with open(restore_grasper_file, "r") as file:
            encoded = ast.literal_eval(file.read())
            grid = Grid.decode_grid(encoded)
    print("Tot samples on grid:", grid.get_tot_samples())


    parser = argparse.ArgumentParser()
    parser.add_argument("--store_session")
    parser.add_argument("--print_samples_pos", action="store_true")
    args = parser.parse_args()
    if args.print_samples_pos:
        grid.print_sample_positions()
    if args.store_session:
        store_samples_on_grid(grid, args.store_session)
        
        # save grid
        grid.write_on_file(restore_grasper_file)
        print("Final samples on grid:", grid.get_tot_samples())
