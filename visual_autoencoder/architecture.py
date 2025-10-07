import torch
from torch import nn
import os
import yaml


DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# load hyperparameters
config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")
with open(config_file) as file:
    CONFIG = yaml.safe_load(file)


""" 
initial top image size: [3, 720, 1280] [batch_size, channels, height, width]
downscale /8 to [3, 90, 160]

Convolutional Encoder architecture
3/4 double convolutional layers with max pooling
and final fully conntected layer to 5D latent space
"""

class EncoderBlock(torch.nn.Module):
    def __init__(self, in_channels, out_channels, pooling=True):
        super(EncoderBlock, self).__init__()

        # architecture
        self.layers = [
            nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(in_channels=out_channels, out_channels=out_channels, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
        ]
        if pooling:
            self.layers.append(nn.MaxPool2d(kernel_size=2, stride=2))
        self.layers = nn.Sequential(*self.layers)

    def forward(self, x):
        return self.layers(x)
        

class ConvolutionalEncoder(torch.nn.Module):
    def __init__(self, 
                 model_name=CONFIG["model"]["name"], 
                 in_channels=3,
                 output_dim=len(CONFIG["model"]["dimensions_to_learn"])):
        super(ConvolutionalEncoder, self).__init__()

        # model name and path
        if model_name is not None:
            self.model_name = model_name
        self.model_path = os.path.join(os.path.dirname(__file__), "convolutional_encoder", self.model_name)
        os.makedirs(self.model_path, exist_ok=True)

        self.in_channels = in_channels
        self.output_dim = output_dim
        # architecture
        # self.encoder = torch.nn.Sequential( # input [3, 45, 80]
        #     EncoderBlock(in_channels=in_channels, out_channels=8),   # [8, 22, 40]
        #     EncoderBlock(in_channels=8, out_channels=16),            # [16, 11, 20]
        #     EncoderBlock(in_channels=16, out_channels=32),           # [32, 5, 10]
        #     EncoderBlock(in_channels=32, out_channels=64, pooling=False),           # [64, 5, 10] no pooling

        #     torch.nn.Flatten(),
            
        #     # fully connected layer to 5D latent space
        #     torch.nn.Linear(in_features=64*5*10, out_features=2), # flatten and reduce dimension
        #     torch.nn.Sigmoid() # to keep outputs between 0 and 1
        # )
        self.encoder = torch.nn.Sequential( # input [3, 45, 80]
            EncoderBlock(in_channels=in_channels, out_channels=4),   # [4, 22, 40]
            EncoderBlock(in_channels=4, out_channels=8),            # [8, 11, 20]
            EncoderBlock(in_channels=8, out_channels=16),           # [16, 5, 10]
            EncoderBlock(in_channels=16, out_channels=32, pooling=False),           # [32, 5, 10] no pooling

            torch.nn.Flatten(),
            
            # fully connected layer to 5D latent space
            torch.nn.Linear(in_features=32*5*10, out_features=self.output_dim), # flatten and reduce dimension
            torch.nn.Sigmoid() # to keep outputs between 0 and 1
        )
        # print(self.encoder)

    def forward(self, x):
        return self.encoder(x)
    
    def save_model(self):
        torch.save(self.state_dict(), os.path.join(self.model_path, "model.pth"))
    
    def load_model(self):
        self.load_state_dict(
            torch.load(os.path.join(self.model_path, "model.pth"), 
            map_location=DEVICE))