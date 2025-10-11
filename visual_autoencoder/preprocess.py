import numpy as np
from torchvision import transforms
import os
from PIL import Image
import torch

import json
import csv

from torch.utils.data import DataLoader
import matplotlib.pyplot as plt

def check_dataset_alignment(dataset, log_data, n=5, shuffle=False):
    """
    Show n random samples from the DataLoader and compare to log file.
    Args:
        dataset: your Dataset object.
        log_data: list of dicts, loaded from your original log file.
        n: how many samples to check.
        shuffle: if True, DataLoader will shuffle (for testing correctness).
    """
    loader = DataLoader(dataset, batch_size=1, shuffle=shuffle)

    print("Comparing dataset samples to log entries:\n")

    for i, (img, label) in enumerate(loader):
        idx = i  # dataset index (adjust if your Dataset uses something else)
        print(f"Sample {i}: Label from DataLoader →", label)
        print("          Original log entry     →", log_data[idx])
        plt.imshow(img[0].permute(1, 2, 0).clip(0, 1))
        plt.title(f"Index {idx}")
        plt.axis("off")
        plt.show()
        if i >= n - 1:
            break


def log_to_csv(log_path, csv_path):
    """
    Converts a log file containing a list of dictionaries into a CSV file.
    Args:
        log_path (str): Path to the .log or .json file.
        csv_path (str): Path to save the .csv file.
    """
    with open(log_path, "r") as f:
        log_data = json.load(f)  # assumes the file is valid JSON (list of dicts)

    # use the keys of the first dict as column headers
    headers = list(log_data[0].keys())

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["index"] + headers)
        writer.writeheader()
        for i, entry in enumerate(log_data):
            writer.writerow({"index": i, **entry})

    print(f"Saved {len(log_data)} entries to {csv_path}")

    return log_data  # return parsed data for further inspection



if __name__ == "__main__":
    path = os.getenv("DATASET_PATH") # experiment path
    log_data = log_to_csv(os.path.join(path,"1/task/states.json"), os.path.join(path,"1/task/states.csv"))



    # preprocess images: resize and convert to tensor
    top_img_shape = np.array([720, 1280])
    resize_factor = 16
    resize_shape = [x for x in map(int,top_img_shape//resize_factor)]
    print(f"Resizing images from {top_img_shape} to {resize_shape}")
    preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize(resize_shape)
            ])

    for fname in os.listdir(path):
        if fname.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff')):
            img_path = os.path.join(path, fname)
            img = Image.open(img_path).convert('RGB')
            img_tensor = preprocess(img)
            out_fname = os.path.splitext(fname)[0] + '_tr.pt'
            torch.save(img_tensor, os.path.join(path, out_fname))

    #         # Convert tensor back to PIL Image for saving
    #         # img_out = transforms.ToPILImage()(img_tensor)
    #         # out_fname = os.path.splitext(fname)[0] + '_tr.png'
    #         # out_path = os.path.join(data_root, out_fname)
    #         # img_out.save(out_path)