import torch
from torchvision import transforms
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
import numpy as np
import cv2
import os
import json


project_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class GripperDataset(Dataset):
    def __init__(self, abs_path=None, experiment=None, sessions=None):
        # get dataset from absolute path or default path and experiment name
        if abs_path is not None:
            data_path = abs_path
        elif experiment is not None:
            data_path = os.path.abspath(os.path.join(project_path, "autograsper", "recorded_data", experiment))
        else:
            raise Exception("Either abs_path or experiment should be provided")
        
        self.transform = transforms.ToTensor()

        self.bottom_images = []
        self.top_images = []
        self.states = []

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

            print(f"Loading data session {session}")
            new_bottom_images = os.listdir(bottom_images_path)
            new_bottom_images = [os.path.join(bottom_images_path, img) for img in new_bottom_images]
            new_top_images = os.listdir(top_images_path)
            new_top_images = [os.path.join(top_images_path, img) for img in new_top_images]
            new_states = load_states(states_path)

            assert (len(new_bottom_images)==len(new_top_images) and len(new_bottom_images)==len(new_states)), f"Mismatch in the number of states, bottom and top images in session {session}"

            self.bottom_images.extend(new_bottom_images)
            self.top_images.extend(new_top_images)
            self.states.extend(new_states)


    def __len__(self):  
        """
        Returns
            length of the dataset
        """
        return len(self.states)

    def __getitem__(self, index):
        """
        Args
            index (int)
        
        Returns
            bottom_image, top_image, state as Tensor """
        bottom_img_path = self.bottom_images[index]
        top_img_path = self.top_images[index]
        state = self.states[index]

        bottom_img = load_image(bottom_img_path)
        top_img = load_image(top_img_path)

        bottom_img = self.transform(bottom_img)
        top_img = self.transform(top_img)
        state_values = torch.tensor(list(map(float, state.values())), dtype=torch.float32)

        return bottom_img, top_img, state_values
    

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
    

def load_image(image_path):
    """
    Args:
        folder_path (str): absolute path of the folder

    Returns:
        np.array of images of shape (num_images, height, width, rgb)
    """

    img = cv2.imread(image_path)
    if img is None:
        raise Exception(f"Image {image_path} not found")
    img = np.transpose(img, (1,2,0))

    return img


def concatenate_images(bottom_img, top_img, space=15):
    if bottom_img.shape[0] != top_img.shape[0]:
        bottom_img, top_img = resize_images(bottom_img, top_img)

    # white space between images
    white_img = np.ones((bottom_img.shape[0], space, 3))

    return np.concatenate([bottom_img, white_img, top_img],axis=1)

def resize_images(bottom_img, top_img, color="RGB"):
    if not isinstance(bottom_img, np.ndarray):
        bottom_img = bottom_img.numpy()
    if not isinstance(top_img, np.ndarray):
        top_img = top_img.numpy()

    height = min(bottom_img.shape[0], top_img.shape[0])
    bottom_img = cv2.resize(bottom_img, (int(bottom_img.shape[1] * height / bottom_img.shape[0]), height))
    top_img = cv2.resize(top_img, (int(top_img.shape[1] * height / top_img.shape[0]), height))

    if color == "RGB":
        # Convert the image from BGR (OpenCV format) to RGB (Matplotlib format)
        bottom_img = cv2.cvtColor(bottom_img, cv2.COLOR_BGR2RGB)
        top_img = cv2.cvtColor(top_img, cv2.COLOR_BGR2RGB)

    return bottom_img, top_img

def plot_images(bottom_img, top_img, title=None):
    fig, ax = plt.subplots(figsize=(15, 5))
    concatenated_img = concatenate_images(bottom_img, top_img)

    ax.imshow(concatenated_img)
    ax.axis('off')
    ax.set_aspect('auto') 
    if title is not None:
        ax.set_title(title, fontsize=18)  
    plt.show()
