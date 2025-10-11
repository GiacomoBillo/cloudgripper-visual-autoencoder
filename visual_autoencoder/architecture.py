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
Convolutional building blocks for encoder and decoder
"""
class DoubleConv(nn.Module):
    def __init__(self, in_channels, out_channels):
        super(DoubleConv, self).__init__()

        # architecture
        self.double_conv = nn.Sequential(
            nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
            nn.Conv2d(in_channels=out_channels, out_channels=out_channels, kernel_size=3, stride=1, padding=1),
            nn.ReLU(),
        )

    def forward(self, x):
        return self.double_conv(x)

class EncoderBlock(torch.nn.Module):
    def __init__(self, in_channels, out_channels, pooling=True):
        super(EncoderBlock, self).__init__()

        self.pool = pooling

        self.double_conv = DoubleConv(in_channels, out_channels)
        if pooling:
            self.pool = nn.MaxPool2d(kernel_size=2, stride=2)

    def forward(self, x):
        x = self.double_conv(x)
        if self.pool:
            x = self.pool(x)
        return x

# class EncoderBlock(torch.nn.Module):
#     def __init__(self, in_channels, out_channels, pooling=True):
#         super(EncoderBlock, self).__init__()

#         # architecture
#         self.layers = [
#             nn.Conv2d(in_channels=in_channels, out_channels=out_channels, kernel_size=3, stride=1, padding=1),
#             nn.ReLU(),
#             nn.Conv2d(in_channels=out_channels, out_channels=out_channels, kernel_size=3, stride=1, padding=1),
#             nn.ReLU(),
#         ]
#         if pooling:
#             self.layers.append(nn.MaxPool2d(kernel_size=2, stride=2))
#         self.layers = nn.Sequential(*self.layers)

#     def forward(self, x):
#         return self.layers(x)
        

class DecoderBlock(nn.Module):
    def __init__(self, in_channels: int, out_channels: int, padding=0, out_padding=0):
        super(DecoderBlock, self).__init__()

        self.up = nn.ConvTranspose2d(in_channels, out_channels, kernel_size=2, stride=2, padding=padding, output_padding=out_padding)
        self.double_conv = DoubleConv(out_channels, out_channels)  

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.up(x)
        x = self.double_conv(x)
        return x



""" 
initial top image size: [3, 720, 1280] [batch_size, channels, height, width]
downscale /8 to [3, 90, 160]

Convolutional Encoder architecture
3/4 double convolutional layers with max pooling
and final fully conntected layer to 5D latent space
"""
class ConvolutionalEncoder(torch.nn.Module):
    def __init__(self, 
                 model_name=CONFIG["model"]["name"], 
                 in_channels=3,
                 output_dim=len(CONFIG["model"]["dimensions_to_learn"])):
        super(ConvolutionalEncoder, self).__init__()

        # model name and path
        if model_name is not None:
            self.model_name = model_name
        self.model_type = "convolutional_encoder"
        self.model_path = os.path.join(os.path.dirname(__file__), self.model_type, self.model_name)
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
        

"""
Deterministic decoder architecture
to use as baseline for comparison with other generative decoders
and to use as first step for 2-step decoders

Architecture:
- configuration embedding with Fourier embedding or MLP
- MLP or CNN to image
"""
class ConvolutionalDecoder(torch.nn.Module):
    def __init__(self, 
                 model_name=CONFIG["model"]["name"], 
                 channels=3,
                 input_dim=len(CONFIG["model"]["dimensions_to_learn"])):
        super(ConvolutionalDecoder, self).__init__()

        # model name and path
        if model_name is not None:
            self.model_name = model_name
        self.model_type = "convolutional_decoder"
        self.model_path = os.path.join(os.path.dirname(__file__), self.model_type, self.model_name)
        os.makedirs(self.model_path, exist_ok=True)

        self.channels = channels
        self.input_dim = input_dim

        # architecture
        self.decoder = torch.nn.Sequential( # input [5]
            # fully connected layer to expand dimension
            torch.nn.Linear(in_features=self.input_dim, out_features=32*5*10), # expand dimension
            torch.nn.ReLU(),
            torch.nn.Unflatten(dim=1, unflattened_size=(32, 5, 10)), # [32, 5, 10]

            DecoderBlock(in_channels=32, out_channels=16, out_padding=(1,0)),           # [16, 11, 20]
            DecoderBlock(in_channels=16, out_channels=8, out_padding=(0,0)),            # [8, 22, 40]
            DecoderBlock(in_channels=8, out_channels=4, out_padding=(1,0)),             # [4, 45, 80]
            DoubleConv(in_channels=4, out_channels=channels),   # [3, 45, 80] no pooling

            nn.Sigmoid() # to keep outputs between 0 and 1
        )
        # print(self.decoder)

    def forward(self, x):
        return self.decoder(x)
    
    def save_model(self):
        torch.save(self.state_dict(), os.path.join(self.model_path, "model.pth"))
    
    def load_model(self):
        self.load_state_dict(
            torch.load(os.path.join(self.model_path, "model.pth"), 
            map_location=DEVICE))
        

if __name__ == "__main__":
    # test encoder and decoder with random input to check dimensions
    x = torch.randn((1, 3, 720//16, 1280//16)).to(DEVICE)
    print("Input shape:", x.shape)
    model = ConvolutionalEncoder(model_name="debug_enc").to(DEVICE)
    output = model(x)
    print("Output shape:", output.shape)  # should be [1, 5]
    decoder = ConvolutionalDecoder(model_name="debug_dec").to(DEVICE)
    reconstructed = decoder(output)
    print("Reconstructed shape:", reconstructed.shape)  # should be [1, 3, 90, 160]

    # summary
    from torchinfo import summary
    summary(model, input_size=(1, 3, 45, 80))
    summary(decoder, input_size=(1, len(CONFIG["model"]["dimensions_to_learn"])))