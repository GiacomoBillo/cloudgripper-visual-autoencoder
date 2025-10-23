import torch
import yaml
from training import Trainer, ARCHITECTURES_WITH_REFERENCE
import os
from utils import get_data
from accelerate import Accelerator # for multigpu
from dotenv import load_dotenv

# import architectures
from architecture import ConvolutionalEncoder, ConvolutionalDecoder, FourierMlpDecoder
from conditioned_UNet import UNetWithFiLM


DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
load_dotenv()  # from .env file


if __name__ == "__main__":
    # load configurations from config.yaml
    config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")
    with open(config_file) as file:
        config = yaml.safe_load(file)
    # get dataset path from .env file
    if config["data"]["dataset_path"] is None:
        config["data"]["dataset_path"] = os.getenv("DATASET_PATH") 
    image_shape = [x//config["data"]["resize_factor"] for x in config["data"]["top_img_shape"]]

    # accelerator for multigpu
    accelerator = Accelerator()

    model_type = config["model"]["type"]
    if model_type == "encoder":
        # create encoder
        model = ConvolutionalEncoder(
            config=config,
            accelerator=accelerator,
        )
        model.summary(input_size=(1, 3, *image_shape))
    elif model_type == "decoder":
        # create decoder
        # model = ConvolutionalDecoder(
        #     config=config,
        #     accelerator=accelerator,
        # )
        # model = FourierMlpDecoder(
        #     config=config,
        #     accelerator=accelerator,
        # )
        # model.summary(input_size=(1, len(config["model"]["dimensions_to_learn"])))

        model = UNetWithFiLM(
            config=config, 
            accelerator=accelerator,
        )
        model.summary(input_size=[(1, 3, *image_shape), (1, len(config["model"]["dimensions_to_learn"]))])
    """
    # dimension example with random input
    x = torch.randn((1, 3, 720//8, 1280//8)).to(DEVICE)
    print("Input shape:", x.shape)
    output = model(x)
    print("Output shape:", output.shape)  # should be [1, 5]
    """

    # dataset and loaders
    config["data"]["num_workers"] = int(os.getenv("NUM_WORKERS", 0)) # set num_workers from .env, default 0
    load_reference = type(model).__name__ in ARCHITECTURES_WITH_REFERENCE  # load reference data for certain architectures
    train_loader, val_loader, test_loader = get_data(config, logger=model.logger, load_reference=load_reference)

    # train model
    trainer = Trainer(model, config)
    trainer.train_model(train_loader, 
                        val_loader=val_loader if config["training"]["validation"] else None, # early stopping if val_loader is provided
                        early_stopping_enabled=config["training"]["early_stopping"],
                        epochs=config["training"]["epochs"]
                        )
