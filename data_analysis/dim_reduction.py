import torch
from torchvision import transforms
from torch.utils.data import DataLoader
from gripper_data import GripperDataset, plot_images, compute_mean_images, load_images
import os
import sys
import numpy as np
from tqdm import tqdm
from sklearn.decomposition import IncrementalPCA

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {DEVICE}")


def incremental_pca(dataloader: Dataloader, n_components=None):
    pca = IncrementalPCA(batch_size=dataloader.batch_size, n_components=n_components)

    for batch in tqdm(dataloader, desc="Fitting PCA", total=len(dataloader)):
        images, states = batch

        # apply incremental PCA on the batch
        pca.partial_fit(top_images - top_mean)


if __name__ == "__main__":
    # resize_shape = 50
    preprocess = transforms.Compose([
            transforms.ToTensor(),
            # transforms.Resize(resize_shape),
            transforms.Lambda(lambda x: torch.flatten(x))
            ])
    batch_size = int(os.getenv("BATCH_SIZE"))
    sessions = np.arange(1,21) # sessions to load
    path = os.getenv("DATASET_PATH") # experiment path
    images_to_load = ["Images"]
    dataset = GripperDataset(abs_path=path, sessions=sessions, transform=preprocess, images_to_load=images_to_load)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    