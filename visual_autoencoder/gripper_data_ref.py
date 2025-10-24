import torch
from torchvision import transforms
from torch.utils.data import Dataset, DataLoader
import matplotlib.pyplot as plt
import numpy as np
import cv2
import os
import json
from tqdm import tqdm
import pandas as pd
from sklearn.cluster import KMeans

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
project_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
import re

def natural_sort_key(s):
    """Return a key for natural sorting (numeric parts as ints)."""
    return [int(t) if t.isdigit() else t.lower() for t in re.split(r'(\d+)', s)]


"""
Create Dataset object
with
- Images (by default "Bottom_images","Images")
    - 1 or more between ["Bottom_images","Images", "Top_masked_images", "Top_masks"]
- States (states.json)
"""
class GripperDatasetReference(Dataset):
    def __init__(self, 
                 abs_path=None, 
                 experiment=None, 
                 sessions=None,
                 transform=None,
                 images_to_load=["Bottom_images","Images"],
                 verbose=False,
                 ):
        # get dataset from absolute path or default path and experiment name
        if abs_path is not None:
            data_path = abs_path
        elif experiment is not None:
            data_path = os.path.abspath(os.path.join(project_path, "autograsper", "recorded_data", experiment))
        else:
            raise Exception("Either abs_path or experiment should be provided")
        
        # NOTE: transform.ToTensor() reshape the image (H,W,C) -> (C,H,W)
        if transform is None:
            transform = transforms.ToTensor()
        self.transform = transform

        # always load states and load images chosen (by default original bottom and top)
        self.images_to_load = images_to_load
        self.images = {}
        for image_type in images_to_load:
            self.images[image_type] = []
        # self.bottom_images = []
        # self.top_images = []
        self.states = []

        # load all recording sessions
        if sessions is None:
            sessions = os.listdir(data_path)

        # load each sessions
        for session in sessions:
            session_path = os.path.join(data_path, str(session), "task")

            # skip JSON files
            if not os.path.isdir(session_path):
                continue
            
            image_path = {}
            for image_type in images_to_load:
                image_path[image_type] = os.path.abspath(os.path.join(session_path, image_type)) 
            # bottom_images_path = os.path.abspath(os.path.join(session_path, "Bottom_Images")) 
            # top_images_path = os.path.abspath(os.path.join(session_path, "Images")) 
            states_path = os.path.abspath(os.path.join(session_path, "states.json")) 

            if verbose:
                print(f"Loading data session {session}")
            new_images = {}
            for image_type in images_to_load:
                # first filter only jpeg files
                new_images[image_type] = sorted([f for f in os.listdir(image_path[image_type]) if f.endswith('.jpeg')], key=natural_sort_key)
                # then add full path
                new_images[image_type] = [os.path.join(image_path[image_type], img) for img in new_images[image_type]]
            # new_bottom_images = os.listdir(bottom_images_path)
            # new_bottom_images = [os.path.join(bottom_images_path, img) for img in new_bottom_images]
            # new_top_images = os.listdir(top_images_path)
            # new_top_images = [os.path.join(top_images_path, img) for img in new_top_images]
            new_states = load_states(states_path)

            # assert that all the folders have the same number of elements
            for lst in new_images.values():
                assert len(lst)==len(new_states), f"Mismatch between the number of states and elements in some image directory, in session {session}"
            # assert (len(new_bottom_images)==len(new_top_images) and len(new_bottom_images)==len(new_states)), f"Mismatch in the number of states, bottom and top images in session {session}"

            for image_type in images_to_load:
                self.images[image_type].extend(new_images[image_type])
            # self.bottom_images.extend(new_bottom_images)
            # self.top_images.extend(new_top_images)
            self.states.extend(new_states)
        
        # --- Load all configurations from CSV (first 5 columns only except skip the index) --- (I keep the original self.states as list of dicts for now)
        # df = pd.read_csv(csv_path)
        # self.states_tensor = torch.tensor(df.iloc[:, 1:6].values, dtype=torch.float32)
        df = pd.DataFrame(self.states)
        self.states_tensor = torch.tensor(df.iloc[:, 0:5].values, dtype=torch.float32)
        self.states_tensor[:,3] = self.states_tensor[:,3] / 180 # normalize rotation angle

        
        # --- Initially, use the first image as reference for all ---
        self.num_references = 1
        self.reference_images = {}
        for image_type in images_to_load:
                self.reference_images[image_type] = [self.images[image_type][0]] * len(self.images[image_type])
        self.reference_states_tensor = self.states_tensor[[0]].repeat(len(self.states), 1)


    def select_reference_indices(self, N=20, normalize=False, random_state=42):
        """
        Selects N representative reference configurations from a CSV file of robot states.

        Args:
            N (int): Number of reference configurations to pick.
            normalize (bool): Whether to normalize each column to [0,1] before clustering.
            random_state (int): Random seed for reproducibility.

        Returns:
            ref_indices (list[int]): Indices of chosen reference configurations.
            centers (ndarray): Cluster centers in normalized state space.
        """

        X = self.states_tensor.numpy()  # shape (num_samples, num_features)
        
        # Normalize each column if desired
        if normalize:
            X_min, X_max = X.min(axis=0), X.max(axis=0)
            X = (X - X_min) / (X_max - X_min + 1e-8)

        # Cluster to find representative configurations
        kmeans = KMeans(n_clusters=N, random_state=random_state, n_init='auto')
        kmeans.fit(X)
        centers = kmeans.cluster_centers_
        labels = kmeans.labels_

        # For each cluster, pick the point closest to its centroid
        ref_indices = []
        for i in range(N):
            cluster_points = np.where(labels == i)[0]
            cluster_data = X[cluster_points]
            if len(cluster_points) == 0:
                continue
            dists = np.linalg.norm(cluster_data - centers[i], axis=1)
            ref_indices.append(cluster_points[np.argmin(dists)])
        ref_indices = np.array(ref_indices)

        print(f"Selected {len(ref_indices)} reference indices.")
        return ref_indices, centers

    def set_references(self, reference_indices = None, num_references=1):
        """
        Given a list of indices (in dataset order) corresponding to reference images,
        assign each sample the *nearest* reference by Euclidean distance in config space.
        """
        self.num_references = num_references
        if reference_indices is None:
            ref_indices,_ = self.select_reference_indices(N=num_references)
            print(f"Automatically selected reference index: {ref_indices.tolist()}")
        else:
            ref_indices = np.array(reference_indices, dtype=torch.long)
        self.ref_indices = ref_indices
        self.ref_states = self.states_tensor[ref_indices]
        ref_indices = torch.tensor(ref_indices, dtype=torch.long)
        ref_indices_np = ref_indices.numpy()

        # Compute nearest reference index for each image
        # Efficient vectorized computation
        # states: [N, 5], ref_states: [R, 5]
        diffs = self.states_tensor.unsqueeze(1) - self.ref_states.unsqueeze(0)
        dists = torch.norm(diffs, dim=2)
        nearest_ref = torch.argmin(dists, dim=1).cpu().numpy()

        # Assign references
        for image_type in self.images_to_load:
            self.reference_images[image_type] = [self.images[image_type][i.item()] for i in ref_indices_np[nearest_ref]]
        self.reference_states_tensor = self.ref_states[nearest_ref]


    def __len__(self):  
        """
        Returns
            length of the dataset
        """
        return len(self.states)
    
    def load_tensor_or_image(self,image_path):
        """
        Loads a pre-transformed tensor if available, otherwise loads and transforms the image.
        """
        tensor_path = image_path.replace(".jpeg", "_tr8.pt")

        if os.path.exists(tensor_path):
            # Directly load tensor (already transformed)
            tensor = torch.load(tensor_path, map_location="cpu",weights_only=False)
            # Ensure it's a tensor and in [C,H,W] format
            if not isinstance(tensor, torch.Tensor):
                raise ValueError(f"Loaded object from {tensor_path} is not a torch.Tensor")
            return tensor.float()
        else:
            # Fallback: load and transform the original image
            img = load_image(image_path)
            return self.transform(img)


    def __getitem__(self, index):
        """
        Args
            index (int)
        
        Returns
            bottom_image, top_image, state as Tensor """
        images = {}
        for image_type in self.images_to_load:
            image_path = self.images[image_type][index]
            # images[image_type] = load_image(image_path)
            # images[image_type] = self.transform(images[image_type])
            images[image_type] = self.load_tensor_or_image(image_path)

        # bottom_img_path = self.bottom_images[index]
        # top_img_path = self.top_images[index]
        # bottom_img = load_image(bottom_img_path)
        # top_img = load_image(top_img_path)
        # bottom_img = self.transform(bottom_img)
        # top_img = self.transform(top_img)

        # state = self.states[index]
        # # load only 5 state values
        # state_values = torch.tensor(list(map(float, state.values())), dtype=torch.float32)[:5] # only the first 5 values
        # state_values[3] = state_values[3] / 180 # normalize rotation angle
        state_values = self.states_tensor[index]
        # return bottom_img, top_img, state_values
        ordered_list_of_images = [images[image_type] for image_type in self.images_to_load]

        reference_images = {}
        for image_type in self.images_to_load:
            ref_image_path = self.reference_images[image_type][index]
            # reference_images[image_type] = load_image(ref_image_path)
            # reference_images[image_type] = self.transform(reference_images[image_type])
            reference_images[image_type] = self.load_tensor_or_image(ref_image_path)
        ordered_list_of_reference_images = [reference_images[image_type] for image_type in self.images_to_load]
        reference_states = self.reference_states_tensor[index]

        return (*ordered_list_of_images, state_values, *ordered_list_of_reference_images, reference_states)
    

    def get_references(self):
        """
        Returns:
            reference images and reference states as Tensors
        """
        reference_images = {}
        for image_type in self.images_to_load:
            # Collect all images for the given indices
            ref_image_paths = [self.reference_images[image_type][i] for i in self.ref_indices]

            # Load and transform all reference images
            reference_images[image_type] = torch.stack([
                self.load_tensor_or_image(p) for p in ref_image_paths
            ])
        
        # Collect corresponding reference states
        reference_states = self.reference_states_tensor[self.ref_indices]

        # Keep consistent ordering across image types
        ordered_list_of_reference_images = [reference_images[image_type] for image_type in self.images_to_load]

        return *ordered_list_of_reference_images, reference_states

    

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
        img: image as numpy array in RGB format (Channels, Height, Width)
    """

    img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    
    if img is None:
        raise Exception(f"Image {image_path} not found")
    
    if len(img.shape) == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    return img.squeeze()


def concatenate_images(bottom_img, top_img, space=15):
    if bottom_img.shape[0] != top_img.shape[0]:
        bottom_img, top_img = resize_images(bottom_img, top_img)

    # white space between images
    if len(bottom_img.shape)==2:
        white_img = np.ones((bottom_img.shape[0], space))
    else:
        white_img = np.ones((bottom_img.shape[0], space, bottom_img.shape[2]))

    return np.concatenate([bottom_img, white_img, top_img],axis=1)

def resize_images(bottom_img, top_img):
    if not isinstance(bottom_img, np.ndarray):
        bottom_img = bottom_img.numpy()
    if not isinstance(top_img, np.ndarray):
        top_img = top_img.numpy()

    height = min(bottom_img.shape[0], top_img.shape[0])
    bottom_img = cv2.resize(bottom_img, (int(bottom_img.shape[1] * height / bottom_img.shape[0]), height))
    top_img = cv2.resize(top_img, (int(top_img.shape[1] * height / top_img.shape[0]), height))

    return bottom_img, top_img


def transpose_channels_last(img):
    if len(img.shape)==3 and img.shape[0]==min(img.shape):
        img = np.transpose(img, (1, 2, 0))
    return img

def transpose_channels_first(img):
    if len(img.shape)==3 and img.shape[2]==min(img.shape):
        img = np.transpose(img, (2, 0, 1))
    return img


def plot_image(image,
               title=None,
               ax=None,
               fontsize=16
               ):
    if ax is None:
        fig, ax = plt.subplots()

    image = transpose_channels_last(image)
    image = np.clip(image, 0, 1)
    
    ax.imshow(image)
    ax.axis('off')
    if title is not None:
        ax.set_title(title, fontsize=fontsize)  
    
def plot_images(bottom_img, top_img, title=None):
    # move RGB channels to the last dimension
    bottom_img = transpose_channels_last(bottom_img)
    top_img = transpose_channels_last(top_img)

    concatenated_img = concatenate_images(bottom_img, top_img)

    fig, ax = plt.subplots(figsize=(15, 5))
    ax.imshow(concatenated_img)
    ax.axis('off')
    ax.set_aspect('auto') 
    if title is not None:
        ax.set_title(title, fontsize=18)  
    plt.show()

def plot_image(image, title=None, ax=None, fontsize=14, cmap=None):
    image = image.squeeze()

    if len(image.shape) == 2:
        cmap = 'gray'
    else:
        # move RGB channels to the last dimension
        image = transpose_channels_last(image)

    if ax is None:
        fig, ax = plt.subplots()
    ax.imshow(image, cmap=cmap)
    ax.axis('off')
    if title is not None:
        ax.set_title(title, fontsize=fontsize)  
    plt.show()


def compute_mean_image(dataset: GripperDatasetReference, name=None):
    """
    Args:
        dataset (GripperDatasetReference)
    
    Returns:
        mean_image (np.array): mean of images
    """
    sum_image = None

    for index in tqdm(range(len(dataset))):
        image, states = dataset[index]

        if sum_image is None:
            sum_image = image.numpy()
        else:
            sum_image += image.numpy()
        
    mean_image = sum_image / len(dataset)
    mean_image = mean_image.squeeze()

    if name is not None:
        # store image
        print(f"Storing mean image {name}")
        store_image(mean_image, name)
    return mean_image

def compute_mean_images(dataloader: DataLoader, path="", name=""):
    """
    Args:
        dataloader (DataLoader)
    
    Returns:
        mean_bottom (np.array): mean of bottom images
        mean_top (np.array): mean of top images
    """
    sum_bottom = None
    sum_top = None

    for batch in tqdm(dataloader):
        bottom_images, top_images, states = batch

        # use GPU if available
        bottom_images = bottom_images.to(DEVICE)
        top_images = top_images.to(DEVICE)

        if sum_bottom is None:
            sum_bottom = bottom_images.numpy().mean(axis=0)
            sum_top = top_images.numpy().mean(axis=0)
        else:
            sum_bottom += bottom_images.numpy().mean(axis=0)
            sum_top += top_images.numpy().mean(axis=0)
        
    mean_bottom = sum_bottom / len(dataloader)
    mean_top = sum_top / len(dataloader)

    # store images
    store_images(mean_bottom, mean_top, path, name)

    return mean_bottom, mean_top


def store_image(img, path):
    """
    Save image with shape (H,W,3) dtype=uint8
    Args
        img : image of shape (H,W,3) or (3,H,W) and dtype uint8 or float32
        path : saving position, it creates the directory if it doesn't exist
    Returns
        True if saved successfully
    """
    dir = os.path.dirname(path)
    os.makedirs(dir, exist_ok=True)

    if isinstance(img, torch.Tensor):
        img = img.numpy()
    img = img.squeeze()
    img = (img * 255).astype(np.uint8)

    # RGB image
    if len(img.shape) == 3:
        img = transpose_channels_last(img)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)    
    return cv2.imwrite(path, img)
    

def store_images(bottom_img, top_img, path="", name=""):
    # move RGB channels to the last dimension
    bottom_img = transpose_channels_last(bottom_img)
    top_img = transpose_channels_last(top_img)

    if path!="" and not os.path.exists(path):
        os.makedirs(path)

    if bottom_img.dtype == np.float32:
        bottom_img = (bottom_img * 255).astype(np.uint8)
    if top_img.dtype == np.float32:
        top_img = (top_img * 255).astype(np.uint8)
    cv2.imwrite(os.path.join(path,f"bottom{name}.jpeg"), bottom_img)
    cv2.imwrite(os.path.join(path,f"top{name}.jpeg"), top_img)


def load_images(path="", name=""):
    bottom_img = cv2.imread(os.path.join(path, f"bottom{name}.jpeg"))
    top_img = cv2.imread(os.path.join(path, f"top{name}.jpeg"))

    # move RGB channels to the first dimension
    bottom_img = transpose_channels_first(bottom_img)
    top_img = transpose_channels_first(top_img) 

    if bottom_img.dtype == np.uint8:
        bottom_img = bottom_img.astype(np.float32) / 255
    if top_img.dtype == np.uint8:
        top_img = top_img.astype(np.float32) / 255

    return bottom_img, top_img

