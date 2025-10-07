import torch
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from gripper_data import GripperDataset

def subsample_dataset(dataset: Dataset, length=None, fraction=None):
    if length is not None:
        assert length > 0 and length <= len(dataset), "Length must be positive and less than or equal to dataset length"
        subset_size = length
    elif fraction is not None:
        assert 0 < fraction <= 1, "Fraction must be between 0 and 1"
        subset_size = int(len(dataset) * fraction)
    else:
        subset_size = len(dataset)  # use full dataset if no len or fraction provided

    indices_subset = list(range(subset_size))
    subset = torch.utils.data.Subset(dataset, indices_subset)
    return subset


def get_data(config, verbose=False):
    top_img_shape = config["data"]["top_img_shape"] # original image shape
    resize_factor = config["data"]["resize_factor"] 
    resize_shape = [int(x//resize_factor) for x in top_img_shape]
    if verbose:
        print(f"Resizing images from {top_img_shape} to {resize_shape}")
    preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize(resize_shape),
            ])
    batch_size = config["training"]["batch_size"]
    # sessions = np.arange(1,21) # sessions to load
    path = config["data"]["dataset_path"] # experiment path
    dataset = GripperDataset(abs_path=path, 
                            #  sessions=sessions, 
                             transform=preprocess,
                             images_to_load=config["data"]["images_to_load"])
    if verbose:
        print(f"Total dataset size: {len(dataset)}")

    torch.manual_seed(6) # for reproducibility
    split = config["data"]["split"] # train, val, test
    train_dataset, val_dataset, test_dataset = torch.utils.data.random_split(dataset, split)
    train_dataset_used = subsample_dataset(train_dataset, length=config["data"]["train_fraction_used"])
    val_dataset_used = subsample_dataset(val_dataset, length=config["data"]["val_fraction_used"])
    if verbose:
        print(f"Len train dataset: {len(train_dataset_used)}, "
          f"Len val dataset: {len(val_dataset_used)}")

    train_loader = DataLoader(train_dataset_used, 
                            batch_size=batch_size, 
                            shuffle=True)
    val_loader = DataLoader(val_dataset_used,
                            batch_size=batch_size)
    test_loader = DataLoader(test_dataset,
                            batch_size=batch_size)

    return train_loader, val_loader, test_loader


def create_model_name(config, verbose=False):
    name = config["model"]["name"] # basename
    if name is None:
        name = ""

    if config["data"]["train_fraction_used"] is not None:
        name += f"_train{config['data']['train_fraction_used']}"
    if config["data"]["val_fraction_used"] is not None:
        name += f"_val{config['data']['val_fraction_used']}"
    if config["training"]["batch_size"] is not None:
        name += f"_batch{config['training']['batch_size']}"
    if config["training"]["learning_rate"] is not None:
        name += f"_lr{config['training']['learning_rate']}"
    if config["training"]["optimizer"] is not None:
        name += f"_{config['training']['optimizer']}"
    # map dimensions to learn indeces to short names
    name += "_"
    dim_map = {0: "x", 1: "y", 2: "z", 3: "r", 4: "g"}
    for dim in config["model"]["dimensions_to_learn"]:
        name += f"{dim_map[dim]}"

    # replace dots
    name = name.replace(".", "_")
    # remove first underscore
    if name.startswith("_"):
        name = name[1:]

    if verbose:
        print(f"Model name: {name}")
    return name