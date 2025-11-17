"""
Baseline evaluations to compare against the Visual Autoencoder.
- nearest neighbor
- random selection
"""

import torch
from tqdm import tqdm
from torchmetrics.image.lpip import LearnedPerceptualImagePatchSimilarity as LPIPSLoss
from torchmetrics.image import PeakSignalNoiseRatio as PSNRLoss, StructuralSimilarityIndexMeasure as SSIMLoss
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
from utils import DIMENSIONS


# Evaluation functions
def MSE_per_dimension(outputs, targets):
    mse = torch.nn.MSELoss(reduction='none')(outputs, targets)
    mse_per_dim = torch.mean(mse, dim=0)  # mean over batch
    return mse_per_dim

ENCODER_EVAL_METRICS = {
    "MSE": torch.nn.MSELoss(),
    "MSE per dimension": MSE_per_dimension,
}

DECODER_EVAL_METRICS = {
    "MSE": torch.nn.MSELoss(),
    "LPIPS": LPIPSLoss(net_type="vgg", normalize=True),
    "SSIM": SSIMLoss(data_range=1.0),
    "PSNR": PSNRLoss(data_range=1.0)
}


def nearest_neighbor_evaluation_encoder(
        all_dataset,
        test_dataset,
        train_dataset,
        eval_metrics=ENCODER_EVAL_METRICS, 
        device=DEVICE,
        batch_size=32,
        verbose=False):
    """
    Perform nearest neighbors evaluation
    1. For each test label, find the nearest neighbor in the training set
    2. Compute evaluation metrics between the test label and its nearest neighbor
    """
    results = {}
    
    keys = ['x_norm', 'y_norm', 'z_norm', 'rotation', 'claw_norm']
    matrix = [[float(state[k]) for k in keys] for state in all_dataset.states]
    all_states = torch.tensor(matrix, dtype=torch.float32)
    all_states[:,3] = all_states[:,3]/180 # normalize rotation

    all_train_indices = train_dataset.indices
    all_train_states = all_states[all_train_indices].to(DEVICE)
    all_test_indices = test_dataset.indices
    all_test_states = all_states[all_test_indices].to(DEVICE)

    # compute pairwise distances between test and all training labels
    # Shape: (batch_test, batch_train)
    dists = torch.cdist(all_test_states, all_train_states)  # Euclidean distance
    # diffs = test_labels.unsqueeze(1) - train_labels_all.unsqueeze(0)
    # dists = torch.norm(diffs, dim=2)
    nearest_labels = torch.argmin(dists, dim=1)

    batch_test_states = []
    batch_nearest_states = []
    # Iterate over test set
    for idx in tqdm(range(0, len(test_dataset), batch_size)):
        batch_test_states = all_test_states[idx:idx+batch_size].to(device)
        nn_indices = nearest_labels[idx:idx+batch_size].to(device)
        batch_nearest_states = all_train_states[nn_indices].to(device)

        # compute metrics between test state and nearest neighbor
        for metric_name, metric_fn in eval_metrics.items():
            score = metric_fn(batch_test_states.clamp(0,1), batch_nearest_states.clamp(0,1))
            
            if metric_name == "MSE per dimension":
                # log each dimension separately
                for dim_idx, dim_loss in enumerate(score):
                    if f"MSE_dim_{DIMENSIONS[dim_idx]}" not in results:
                        results[f"MSE_dim_{DIMENSIONS[dim_idx]}"] = []
                    results[f"MSE_dim_{DIMENSIONS[dim_idx]}"].append(dim_loss.item())
            else:
                if metric_name not in results:
                    results[metric_name] = []
                results[metric_name].append(score.item())

    # average results
    averaged_results = {
        metric: sum(values) / len(values) for metric, values in results.items() 
    }
    mse_keys = [key for key in averaged_results.keys() if "MSE" in key]
    for key in mse_keys:
        averaged_results["R"+key] = averaged_results[key]**0.5
    
    if verbose:
        print("Nearest Neighbor Decoder Evaluation Results:")
        for key, val in averaged_results.items():
            print(f"{key} loss: {val:.4f}")
    return averaged_results


def nearest_neighbors_evaluation_decoder(
        all_dataset,
        test_dataset,
        train_dataset,
        eval_metrics=DECODER_EVAL_METRICS, 
        device=DEVICE,
        batch_size=32,
        verbose=False):
    """
    Perform nearest neighbors evaluation
    1. For each test image, find the nearest neighbor in the training set
    2. Compute evaluation metrics between the test image and its nearest neighbor
    """
    
    results = {metric_name: [] for metric_name in eval_metrics}    

    keys = ['x_norm', 'y_norm', 'z_norm', 'rotation', 'claw_norm']
    matrix = [[float(state[k]) for k in keys] for state in all_dataset.states]
    all_states = torch.tensor(matrix, dtype=torch.float32)
    all_states[:,3] = all_states[:,3]/180 # normalize rotation

    train_indices = train_dataset.indices
    train_states = all_states[train_indices].to(DEVICE)
    test_indices = test_dataset.indices
    test_states = all_states[test_indices].to(DEVICE)

    # compute pairwise distances between test and all training labels
    # Shape: (batch_test, batch_train)
    dists = torch.cdist(test_states, train_states)  # Euclidean distance
    # diffs = test_labels.unsqueeze(1) - train_labels_all.unsqueeze(0)
    # dists = torch.norm(diffs, dim=2)
    nearest_labels = torch.argmin(dists, dim=1)

    test_images = []
    nearest_images = []
    # Iterate over test set
    for idx in tqdm(range(len(test_dataset))):
        # load test image
        test_image, test_state = test_dataset[idx]
        test_image = test_image.to(device)
        # load nearest neighbor image from training set
        nn_index = nearest_labels[idx]
        nearest_image, nearest_state = train_dataset[nn_index]
        nearest_image = nearest_image.to(device)

        test_images.append(test_image.unsqueeze(0))
        nearest_images.append(nearest_image.unsqueeze(0))

        if idx % batch_size == 0 or idx == len(test_dataset)-1:
            test_images = torch.cat(test_images, dim=0)
            nearest_images = torch.cat(nearest_images, dim=0)

            # compute metrics between test image and nearest neighbor
            for metric_name, metric_fn in eval_metrics.items():
                score = metric_fn(test_images.clamp(0,1), nearest_images.clamp(0,1))
                if torch.is_tensor(score):
                    score = score.item()
                results[metric_name].append(score)

            test_images = []
            nearest_images = []

    # average results
    averaged_results = {
        metric: sum(values) / len(values) for metric, values in results.items()
    }
    averaged_results["RMSE"] = averaged_results["MSE"]**0.5
    if verbose:
        print("Nearest Neighbor Encoder Evaluation Results:")
        for key, val in averaged_results.items():
            print(f"{key} loss: {val:.4f}")
    return averaged_results