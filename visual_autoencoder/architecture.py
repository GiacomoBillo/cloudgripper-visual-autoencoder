import torch
from torch import nn
import os
import torchinfo
import yaml
from accelerate import Accelerator, load_checkpoint_in_model # for multigpu
from abc import ABC, abstractmethod # abstract class
import re
from utils import create_model_name
from dotenv import load_dotenv
from utils import Logger

load_dotenv()  # from .env file
VERBOSE = os.getenv("VERBOSE", "False").lower() == "true"

# load hyperparameters
config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")
with open(config_file) as file:
    CONFIG = yaml.safe_load(file)

CHANNELS = 3

"""
Abstract base class for all architectures
"""
class BaseArchitecture(nn.Module, ABC):
    def __init__(self, 
                 model_name=None,
                 config=None,
                 load_model=False,
                ):
        super().__init__()  

        if model_name is None and config is None:
            raise ValueError("Model name or config must be provided")
        
        # model name, type and path
        if model_name is None:
            self.model_name = create_model_name(config)
        else:
            self.model_name = model_name
        self.model_type = re.sub(r'(?<!^)(?=[A-Z])', '_', self.__class__.__name__).lower()
        self.model_path = os.path.join(os.path.dirname(__file__), self.model_type, self.model_name)
        os.makedirs(self.model_path, exist_ok=True)
        self.logger = Logger(self.model_path, print_on_console=VERBOSE)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # if load model, load weights and config
        if load_model:
            self.load_model()
            self.config = self.load_config() 
        # if new model, use provided config
        else:
            self.config = config
            self.save_config()
            self.logger.print(f"Model name: {self.model_name}\n")

        self.channels = CHANNELS


    @abstractmethod
    def forward(self, x):
        # return self.architecture(x)
        pass
    
    # save model non-accelerated
    def save_model(self):
        torch.save(self.state_dict(), os.path.join(self.model_path, "model.pth"))
    
    # load model non-accelerated
    def load_model(self):
        if not os.path.exists(os.path.join(self.model_path, "model.pth")):
            raise FileNotFoundError(f"No model file found in {self.model_path}")
        self.load_state_dict(
            torch.load(os.path.join(self.model_path, "model.pth"), 
            map_location=self.device))

    def save_config(self):
        # save config to model path
        with open(os.path.join(self.model_path, "config.yaml"), 'w') as file:
            yaml.dump(self.config, file)

    def load_config(self):
        # check if config file exists
        if not os.path.exists(os.path.join(self.model_path, "config.yaml")):
            # raise FileNotFoundError(f"No config file found in {self.model_path}")
            return None
        
        # load config from model path
        with open(os.path.join(self.model_path, "config.yaml"), "r") as file:
            config = yaml.safe_load(file)
        return config
    
    def get_config(self):
        return self.config

    def summary(self, input_size):
        self.logger.print("Model architecture:")
        info = torchinfo.summary(self, input_size=input_size, verbose=0)
        self.logger.print(info)


"""
Abstract base class for architectures with accelerator support
"""
class AcceleratedArchitecture(BaseArchitecture, ABC):
    def __init__(self, 
                 model_name=None,
                 config=None,
                 accelerator: Accelerator=None,
                ):
        super().__init__(model_name, config)

        # accelerator for multigpu
        self.accelerator = accelerator 
        self.device = accelerator.device
    
    # save accelerated model
    def save_model(self):
        if self.accelerator is None:
            super().save_model()
            return
        # wait and synchronize for multigpu
        self.accelerator.wait_for_everyone()
        # unwrap model from accelerator and save
        self.accelerator.save_model(self, 
                                    os.path.join(self.model_path), 
                                    safe_serialization=True)
    
    # load accelerated model
    def load_model(self):
        if self.accelerator is None:
            super().load_model()
            return
        # load accelerated model
        load_checkpoint_in_model(self, 
                                 os.path.join(self.model_path), 
                                 # device_map={"":self.device} # not working?
                                 )

        

    
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
class ConvolutionalEncoder(AcceleratedArchitecture):
    def __init__(self, 
                 model_name=None,
                 config=None,
                 load_model=False,
                 accelerator: Accelerator=None,
                ):
        super().__init__(model_name, config, accelerator)

        self.output_dim = len(self.config["model"]["dimensions_to_learn"])
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
            EncoderBlock(in_channels=self.channels, out_channels=4),   # [4, 22, 40]
            EncoderBlock(in_channels=4, out_channels=8),            # [8, 11, 20]
            EncoderBlock(in_channels=8, out_channels=16),           # [16, 5, 10]
            EncoderBlock(in_channels=16, out_channels=32, pooling=False),           # [32, 5, 10] no pooling

            torch.nn.Flatten(),
            
            # fully connected layer to 5D latent space
            torch.nn.Linear(in_features=32*5*10, out_features=self.output_dim), # flatten and reduce dimension
            torch.nn.Sigmoid() # to keep outputs between 0 and 1
        )
        # print(self.encoder)

        if load_model:
            self.load_model()

    def forward(self, x):
        return self.encoder(x)
        

"""
Deterministic decoder architecture
to use as baseline for comparison with other generative decoders
and to use as first step for 2-step decoders

Architectures:
- Convolutional decoder = MLP + CNN
- Fourier decoder = Fourier embedding + MLP
"""
class ConvolutionalDecoder(AcceleratedArchitecture):
    def __init__(self, 
                 model_name=None,
                 config=None, 
                 load_model=False,
                 accelerator: Accelerator=None,
                ):
        super().__init__(model_name, config, accelerator)

        self.input_dim = len(self.config["model"]["dimensions_to_learn"])

        # architecture
        self.decoder = torch.nn.Sequential( # input [5]
            # fully connected layer to expand dimension
            torch.nn.Linear(in_features=self.input_dim, out_features=32*5*10), # expand dimension
            torch.nn.ReLU(),
            torch.nn.Unflatten(dim=1, unflattened_size=(32, 5, 10)), # [32, 5, 10]

            DecoderBlock(in_channels=32, out_channels=16, out_padding=(1,0)),           # [16, 11, 20]
            DecoderBlock(in_channels=16, out_channels=8, out_padding=(0,0)),            # [8, 22, 40]
            DecoderBlock(in_channels=8, out_channels=4, out_padding=(1,0)),             # [4, 45, 80]
            DoubleConv(in_channels=4, out_channels=self.channels),   # [3, 45, 80] no pooling

            nn.Sigmoid() # to keep outputs between 0 and 1
        )
        # print(self.decoder)

        if load_model:
            self.load_model()

    def forward(self, x):
        return self.decoder(x)
        

class FourierMlpDecoder(AcceleratedArchitecture):
    def __init__(self, 
                 model_name=None, 
                 config=None,
                 load_model=False,
                 accelerator: Accelerator=None,
                 ):
        super().__init__(model_name, config, accelerator)

        self.input_dim = len(config["model"]["dimensions_to_learn"])
        self.output_shape = [x//config["data"]["resize_factor"] for x in config["data"]["top_img_shape"]] # [height, width]
        self.height, self.width = self.output_shape

        # 5D config -> fourier embedding
        num_frequencies = 32
        self.fourier_embedding = self.FourierEmbedding(
            input_dim=self.input_dim,
            num_frequencies=num_frequencies,
            type="RFF",
            device=accelerator.device
        ) # input [5] -> output [5*2*num_frequencies]

        # MLP to image
        self.mlp = torch.nn.Sequential( 
            torch.nn.Linear(in_features=self.input_dim*2*num_frequencies, out_features=512), # expand dimension
            torch.nn.ReLU(),
            torch.nn.Linear(in_features=512, out_features=self.channels*self.height*self.width),
            torch.nn.Sigmoid(), # to keep outputs between 0 and 1
            torch.nn.Unflatten(dim=1, unflattened_size=(self.channels, self.height, self.width)) # [3, H, W]
        )
        # print(self.decoder)

        if load_model:
            self.load_model()

    def forward(self, x):
        embedding = self.fourier_embedding(x) # [batch, input_dim, num_frequencies*2]
        flatten_embedding = embedding.view(embedding.shape[0], -1)  # [batch, input_dim * num_frequencies * 2]
        # print(f"embedding shape: {embedding.shape}, flatten shape: {flatten_embedding.shape}")
        return self.mlp(flatten_embedding)
    
    class FourierEmbedding(nn.Module):
        # https://arxiv.org/html/2502.05482v1#S4.F4
        def __init__(self, input_dim, num_frequencies, type="RFF", device=torch.device("cuda" if torch.cuda.is_available() else "cpu")):
            super().__init__()
            
            self.input_dim = input_dim
            self.num_frequencies = num_frequencies
            self.type = type
            self.device = device

            # Positional  Encoding (PE)
            if type == "PE":
                scale = 2.0
                frequencies = scale ** torch.linspace(0, num_frequencies - 1, num_frequencies)
            # Random Fourier Features (RFF)
            elif type == "RFF":
                frequencies = torch.randn(( num_frequencies))
            else:
                raise ValueError("Invalid Fourier embedding type")

            # save frequencies to self.frequencies as fixed parameters
            self.register_buffer('frequencies', frequencies)

        # Apply Fourier feature mapping
        def __call__(self, x):
            """
            x: [batch, input_dim]
            Returns: [batch, input_dim, num_frequencies * 2]
            """
            x_proj = 2 * torch.pi * x.unsqueeze(-1) * self.frequencies
            embedding = torch.cat([torch.sin(x_proj), torch.cos(x_proj)], dim=-1)
            return embedding


if __name__ == "__main__":
    # test encoder and decoder with random input to check dimensions
    x = torch.randn((1, 3, 720//16, 1280//16))
    print("Input shape:", x.shape)
    encoder = ConvolutionalEncoder(model_name="debug_enc")
    output = encoder(x)
    print("Output shape:", output.shape)  # should be [1, 5]

    decoder = ConvolutionalDecoder(model_name="debug_dec")
    reconstructed = decoder(output)
    print("Reconstructed shape:", reconstructed.shape)

    # summary
    from torchinfo import summary
    summary(encoder, input_size=(1, 3, 45, 80))
    summary(decoder, input_size=(1, len(CONFIG["model"]["dimensions_to_learn"])))
