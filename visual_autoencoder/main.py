import torch
from torchinfo import summary
from architecture import ConvolutionalEncoder
import yaml
from training import Trainer
import os
from utils import get_data, create_model_name


DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
VERBOSE = True


if __name__ == "__main__":
    # load hyperparameters
    config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")
    with open(config_file) as file:
        config = yaml.safe_load(file)

    # create model name
    model_name = create_model_name(config, verbose=VERBOSE)

    # create model
    model = ConvolutionalEncoder(
        model_name=model_name,
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
    train_loader, val_loader, test_loader = get_data(config, verbose=VERBOSE)
    
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