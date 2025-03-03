import sys
import os
import json
import cv2
import numpy as np
from tqdm import tqdm

project_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def load_images(folder_path, description=""):
    """
    Args:
        folder_path (str): absolute path of the folder

    Returns:
        np.array of images of shape (num_images, height, width, rgb)
    """
    img_list = []

    for img_path in tqdm(os.listdir(folder_path), desc=description):
        img = cv2.imread(os.path.join(folder_path, img_path))
        h, w = img.shape[:2]
        img = cv2.resize(img, (h,w))

        img_list.append(img)

    return np.array(img_list)


def load_states(states_path):
    """
    Read sequence of states from JSON file

    Args:
        states_path (str): absolute path of state file
    
    Returns:
        content of the JSON file, list of dictionaries
    """
    if os.path.exists(states_path):
            with open(states_path, "r") as file:
                return json.load(file)
    else :
        raise Exception(f"File {states_path} not found")


def load_data(experiment: str, sessions=None, verbose=False):
    bottom_images = None
    top_images = None
    states = []
    data_path = os.path.abspath(os.path.join(project_path, "autograsper", "recorded_data", experiment))

    # load all recording sessions
    if sessions is None:
        sessions = os.listdir(data_path)

    # load each sessions
    for session in sessions:
        session_path = os.path.join(data_path, session, "task")

        # skip JSON files
        if not os.path.isdir(session_path):
            continue

        bottom_images_path = os.path.abspath(os.path.join(session_path, "Bottom_Images")) 
        top_images_path = os.path.abspath(os.path.join(session_path, "Images")) 
        states_path = os.path.abspath(os.path.join(session_path, "states.json")) 

        print(f"Loading images session {session}")
        new_bottom_images = load_images(bottom_images_path)
        new_top_images = load_images(top_images_path)
        new_states = load_states(states_path)

        if verbose:
            print(f"Bottom images -> type: {type(new_bottom_images)}, len: {new_bottom_images.shape}")
            print(f"Top images -> type: {type(new_top_images)}, len: {new_top_images.shape}")
            print(f"States -> type: {type(new_states)}, len: {len(new_states)}\n")
        assert (new_bottom_images.shape[0]==new_top_images.shape[0] and new_bottom_images.shape[0]==len(new_states)), f"The number of states, bottom and top samples in session {session} don't match"

        # add session data to dataset
        if bottom_images is None:
            bottom_images = new_bottom_images
        else:
            bottom_images = np.vstack(bottom_images, new_bottom_images)
        if top_images is None:
            top_images = new_top_images
        else:
            top_images = np.vstack(top_images, new_top_images)
        states.extend(new_states)

    return bottom_images, top_images, states 
        
        

if __name__ == "__main__":
    load_data("data_collection_clean_env", sessions=["1"], verbose=True)


