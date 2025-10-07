import torch
from torchinfo import summary
from gripper_data import GripperDataset
from torchvision import transforms
from torch.utils.data import DataLoader, Dataset
from architecture import ConvolutionalEncoder
import yaml
from training import Trainer
import os


DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
VERBOSE = True


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


def get_data(config):
    top_img_shape = config["data"]["top_img_shape"] # original image shape
    resize_factor = config["data"]["resize_factor"] 
    resize_shape = [int(x//resize_factor) for x in top_img_shape]
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
    print(f"Total dataset size: {len(dataset)}")

    torch.manual_seed(6) # for reproducibility
    split = config["data"]["split"] # train, val, test
    train_dataset, val_dataset, test_dataset = torch.utils.data.random_split(dataset, split)
    train_dataset_used = subsample_dataset(train_dataset, length=config["data"]["train_fraction_used"])
    val_dataset_used = subsample_dataset(val_dataset, length=config["data"]["val_fraction_used"])
    print(f"Len train dataset: {len(train_dataset_used)}, Len val dataset: {len(val_dataset_used)}")

    train_loader = DataLoader(train_dataset_used, 
                            batch_size=batch_size, 
                            shuffle=True)
    val_loader = DataLoader(val_dataset_used,
                            batch_size=batch_size)
    test_loader = DataLoader(test_dataset,
                            batch_size=batch_size)

    return train_loader, val_loader, test_loader


if __name__ == "__main__":
    # load hyperparameters
    config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")
    with open(config_file) as file:
        config = yaml.safe_load(file)

    # create model
    model = ConvolutionalEncoder(
        output_dim=len(config["model"]["dimensions_to_learn"])
    ).to(DEVICE)
    summary(model, input_size=(1, 3, 45, 80))

    """
    # dimension example with random input
    x = torch.randn((1, 3, 720//8, 1280//8)).to(DEVICE)
    print("Input shape:", x.shape)
    output = model(x)
    print("Output shape:", output.shape)  # should be [1, 5]
    """

    # dataset
    train_loader, val_loader, test_loader = get_data(config)
    
    """
    # plot one downscaled image
    import matplotlib.pyplot as plt
    image_np = dataset[0][0].permute(1, 2, 0).numpy()
    plt.imshow(image_np)
    plt.axis('off')  # Hide axis
    plt.show()
    """

    # train model
    trainer = Trainer(model, config)
    print("Training model...")
    trainer.train_model(train_loader, 
                        val_loader=val_loader if config["training"]["validation"] else None, # early stopping if val_loader is provided
                        early_stopping_enabled=config["training"]["early_stopping"],
                        epochs=config["training"]["epochs"],
                        verbose=VERBOSE)
    print("Training complete")