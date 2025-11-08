import torch
import torch.nn as nn
from torchinfo import summary
import os
import torch.nn.functional as F
from typing import Optional, Tuple, List, Dict
from architecture import AcceleratedArchitecture
from accelerate import Accelerator
import yaml
from training import Trainer
from utils import get_data
import cv2
from gripper_data import transpose_channels_first
from torchvision.transforms import Resize
import numpy as np
from gripper_data import store_image


""" 
initial top image size: [3, 720, 1280] [batch_size, channels, height, width]
downscale /resize_factor to [3, 720/resize_factor, 1280/resize_factor]
"""


class ConvBlock(nn.Module):
    """
    Two convs with GroupNorm + ReLU.
    We'll apply FiLM modulation (per-channel scale and shift) after the GroupNorm
    and before the ReLU when a modulation is provided.
    """
    def __init__(self, in_ch: int, out_ch: int, groups: int = 8):
        super().__init__()
        self.conv1 = nn.Conv2d(in_ch, out_ch, kernel_size=3, padding=1, bias=False)
        self.gn1 = nn.GroupNorm(num_groups=min(groups, out_ch), num_channels=out_ch)
        self.conv2 = nn.Conv2d(out_ch, out_ch, kernel_size=3, padding=1, bias=False)
        self.gn2 = nn.GroupNorm(num_groups=min(groups, out_ch), num_channels=out_ch)
        self.out_ch = out_ch

    def forward(self, x: torch.Tensor, gamma_beta: Optional[Tuple[torch.Tensor, torch.Tensor]] = None):
        """
        gamma_beta: tuple(gamma, beta) or None
          gamma, beta are tensors of shape [B, C] to be broadcasted to [B, C, H, W]
        """
        x = self.conv1(x)
        x = self.gn1(x)
        if gamma_beta is not None:
            gamma, beta = gamma_beta
            # gamma,beta: [B, C] -> [B, C, 1, 1]
            x = gamma.unsqueeze(-1).unsqueeze(-1) * x + beta.unsqueeze(-1).unsqueeze(-1)
        x = F.relu(x, inplace=True)

        x = self.conv2(x)
        x = self.gn2(x)
        if gamma_beta is not None:
            gamma, beta = gamma_beta
            x = gamma.unsqueeze(-1).unsqueeze(-1) * x + beta.unsqueeze(-1).unsqueeze(-1)
        x = F.relu(x, inplace=True)
        return x


class Down(nn.Module):
    """Downscaling with maxpool then ConvBlock"""
    def __init__(self, in_ch: int, out_ch: int, groups: int = 8):
        super().__init__()
        self.pool = nn.MaxPool2d(2)
        self.block = ConvBlock(in_ch, out_ch, groups=groups)

    def forward(self, x: torch.Tensor, gamma_beta: Optional[Tuple[torch.Tensor, torch.Tensor]] = None):
        x = self.pool(x)
        x = self.block(x, gamma_beta)
        return x


class Up(nn.Module):
    """Upscaling then ConvBlock. Uses nearest upsample + conv"""
    def __init__(self, in_ch: int, out_ch: int, groups: int = 8):
        # in_ch is channels from concat(left, skip) i.e. decoder_channels + skip_channels
        super().__init__()
        self.up = nn.Upsample(scale_factor=2, mode='nearest')
        self.conv1x1 = nn.Conv2d(in_ch // 2 + in_ch // 2, in_ch // 2, kernel_size=1)  # no-op but keeps structure
        # After concat we pass through ConvBlock. We'll set ConvBlock appropriately in UNet construct.
        self.block = ConvBlock(in_ch, out_ch, groups=groups)

    def forward(self, x: torch.Tensor, skip: torch.Tensor, gamma_beta: Optional[Tuple[torch.Tensor, torch.Tensor]] = None):
        x = self.up(x)
        # if shapes mismatch due to odd sizes, pad
        if x.shape[-2:] != skip.shape[-2:]:
            x = F.interpolate(x, size=skip.shape[-2:], mode='nearest')
        x = torch.cat([skip, x], dim=1)  # concat along channels
        x = self.block(x, gamma_beta)
        return x


class ConfigMLP(nn.Module):
    """
    Map the configuration vector to per-block FiLM parameters.
    We'll produce for each block a pair (gamma, beta) of shape [B, channels].
    The architecture creates one head per block for simplicity.
    """
    def __init__(self, config_dim: int, block_channels: List[int], hidden_dim: int = 256, use_layernorm: bool = True):
        super().__init__()
        self.shared = nn.Sequential(
            nn.Linear(config_dim, hidden_dim),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(inplace=True),
        )
        # heads: for each block produce 2 * channels outputs
        self.heads = nn.ModuleList([
            nn.Linear(hidden_dim, ch * 2) for ch in block_channels
        ])
        # optionally layer norm on hidden (keeps outputs stable)
        self.use_layernorm = use_layernorm
        if use_layernorm:
            self.ln = nn.LayerNorm(hidden_dim)

    def forward(self, config: torch.Tensor) -> List[Tuple[torch.Tensor, torch.Tensor]]:
        # config: [B, config_dim]
        h = self.shared(config)  # [B, hidden_dim]
        if self.use_layernorm:
            h = self.ln(h)
        outs = []
        for head in self.heads:
            out = head(h)  # [B, ch*2]
            b, twoch = out.shape
            ch = twoch // 2
            gamma = out[:, :ch]
            beta = out[:, ch:]
            # It's often beneficial to init gamma around 1 and beta around 0.
            # We don't enforce it here, but you can scale or use different activations.
            outs.append((gamma, beta))
        return outs


"""
Conditioned-UNet for image reconstruction from robot configuration, given image reference
The UNet 
- takes the reference image as input
- the robot configuration is injected at each step via FILM layers
- outputs the reconstructed image
https://arxiv.org/abs/1907.01277 
"""
class UNetWithFiLM(AcceleratedArchitecture):
    """
    UNet where conditioning vector is mapped via ConfigMLP to FiLM parameters that are applied
    at every ConvBlock (encoder and decoder).
    """
    def __init__(self,
                 model_name=None, 
                 config=None,
                 load_model=False,
                 accelerator: Accelerator=None,
                 in_channels: int = 3,
                 config_dim: int = 5,
                 base_channels: int = 4,
                 num_downs: int = 4,
                 groups: int = 8,
                 hidden_dim_mlp: int = 256,
                 out_channels: int = 3,
                 ) -> None:
        """
        in_channels: channels of reference input (e.g., 3 RGB; or 6 if ref+clean background stacked)
        config_dim: dimension of the configuration vector
        base_channels: number of filters at first layer (doubling every downsample)
        num_downs: number of downsamplings (depth of the U)
        """
        super().__init__(model_name, config, accelerator)

        self.in_channels = in_channels
        self.config_dim = config_dim
        self.base_channels = base_channels
        self.num_downs = num_downs

        if config is not None:
            self.delta = config["data"].get("delta", False)  # whether to use relative positions

        # build encoder channel sizes
        enc_channels = [base_channels * (2 ** i) for i in range(num_downs + 1)]  # includes bottleneck
        # encoder blocks: first block is input conv (no pooling), then Down blocks
        # We'll implement encoder as: first ConvBlock(in_channels -> base), then Down blocks
        self.enc0 = ConvBlock(in_channels, enc_channels[0], groups=groups)
        self.downs = nn.ModuleList()
        for i in range(1, len(enc_channels)):
            self.downs.append(Down(enc_channels[i-1], enc_channels[i], groups=groups))

        # decoder channel sizes (reverse, except last to produce out_channels)
        dec_channels = list(reversed(enc_channels[:-1]))  # skip last (bottleneck) in reversed list
        # Create Ups where input channels = decoder_in_ch + skip_ch
        self.ups = nn.ModuleList()
        for i in range(len(dec_channels)):
            in_ch = enc_channels[-1 - i] + dec_channels[i]  # previous decoded channels + skip channels
            # but more straightforward: we know the shapes for the Up ConvBlock: in_ch = decoded_channels + skip_channels
            out_ch = dec_channels[i]
            self.ups.append(Up(in_ch, out_ch, groups=groups))

        # bottleneck conv block (enc_channels[-1] -> enc_channels[-1])
        self.bottleneck = ConvBlock(enc_channels[-1], enc_channels[-1], groups=groups)

        # final conv to map to output channels
        self.final_conv = nn.Sequential(
            nn.Conv2d(enc_channels[0], base_channels, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(base_channels, out_channels, kernel_size=1)
        )

        # Prepare ConfigMLP. It needs to produce FiLM for each ConvBlock in sequence:
        # blocks = [enc0] + [each down.block] + [bottleneck] + [each up.block]
        # We'll list the channel sizes for each block to create corresponding heads.
        block_channels = []
        block_channels.append(enc_channels[0])  # enc0
        for ch in enc_channels[1:]:
            block_channels.append(ch)  # each down.block out channels
        block_channels.append(enc_channels[-1])  # bottleneck
        # for ups, each Up.block has out_ch = dec_channels[i]
        for out_ch in dec_channels:
            block_channels.append(out_ch)

        self.config_mlp = ConfigMLP(config_dim=config_dim, block_channels=block_channels, hidden_dim=hidden_dim_mlp)


    def forward(self, ref_img: torch.Tensor, config: torch.Tensor) -> torch.Tensor:
        """
        ref_img: [B, C, H, W]
        config: [B, config_dim]
        returns: [B, out_channels, H, W]
        """
        # produce modulation parameters
        films = self.config_mlp(config)  # list of (gamma, beta) for every ConvBlock in order

        # Encoder
        films_iter = iter(films)
        x0 = self.enc0(ref_img, next(films_iter))  # first block
        skips = [x0]
        x = x0
        for down in self.downs:
            x = down(x, next(films_iter))
            skips.append(x)

        # bottleneck
        x = self.bottleneck(x, next(films_iter))

        # Decoder: iterate ups and pops from skips in reverse
        # available skips: [enc0, down1_out, down2_out, ... , bottleneck_in?] we constructed them above
        # We used all downs' outputs as skips (skips includes the output after each block).
        # For decoding we need corresponding skip feature for each up.
        # Note: len(ups) == len(skips) - 1 (we don't use the last skip which is the deepest)
        skips.pop()  # remove the deepest skip (last one)
        for up in self.ups:
            # pop the last skip that corresponds to the same spatial size
            skip = skips.pop()  # deepest skip
            # But after pooling, skip ordering: skips[0] is shallowest, last is deepest before bottleneck.
            # We popped deepest, so next pop should be next shallower etc.
            x = up(x, skip, next(films_iter))

        out = self.final_conv(x)
        return out
    

class UNetWithFiLMAndEnv(UNetWithFiLM):
    """
    UNet that takes as input the reference image and the background image,
    the 5D robot configuration is injected at each step via FILM layers,
    and outputs the reconstructed image.
    """
    def __init__(self,
                 model_name=None, 
                 config=None,
                 load_model=False,
                 accelerator: Accelerator=None,
                 in_channels: int = 6,  # ref + background
                 config_dim: int = 5,
                 base_channels: int = 4,
                 num_downs: int = 4,
                 groups: int = 8,
                 hidden_dim_mlp: int = 256,
                 out_channels: int = 3,
                 ) -> None:
        super().__init__(model_name, config, load_model, accelerator,
                         in_channels, config_dim, base_channels,
                         num_downs, groups, hidden_dim_mlp, out_channels)

        self.image_shape = [x//config["data"]["resize_factor"] for x in config["data"]["top_img_shape"]] # [height, width]

        # load background/mean image
        self.background_path = os.path.join(self.model_path, "background.jpeg")
        if os.path.exists(self.background_path):
            self.background_image = self.load_background_image()
        else:
            self.background_image = None


    def load_external_background_image(self):
        # Implement loading of background image
        background_image_path = os.getenv("BACKGROUND_IMAGE_PATH")
        if background_image_path is None:
            raise ValueError("BACKGROUND_IMAGE_PATH not set in environment variables.")
        if not os.path.exists(background_image_path):
            raise FileNotFoundError(f"Background image not found at {background_image_path}")
        
        self.background_path = background_image_path
        self.load_background_image()
        
    
    def load_background_image(self):
        background_image = cv2.imread(self.background_path)
        background_image = cv2.cvtColor(background_image, cv2.COLOR_BGR2RGB)  # convert BGR to RGB
        background_image = transpose_channels_first(background_image) # move RGB channels to the first dimension
        if background_image.dtype == np.uint8:
            background_image = background_image.astype(np.float32) / 255.0  # normalize to [0, 1]
        background_image = torch.tensor(background_image, device=self.accelerator.device).unsqueeze(0)  # [1, C, H, W]        
        background_image = Resize(self.image_shape)(background_image)  # downscale if needed
        self.background_image = background_image  # [1, C, H, W]
    

    def create_background_image(self, training_dataset, batch=200):
        # compute 
        self.logger.print("Computing background image from training dataset")
        train_loader = torch.utils.data.DataLoader(training_dataset, batch_size=batch, shuffle=True)
        images, *_ = next(iter(train_loader))  # get one batch
        images = images.to(self.accelerator.device)

        median_img = torch.median(images, dim=0).values.cpu()         
        # save
        store_image(median_img, self.background_path)
        # load and preprocess
        self.load_background_image()


    def forward(self, ref_img: torch.Tensor, config: torch.Tensor) -> torch.Tensor:
        """
        ref_img: [B, C, H, W]
        config: [B, config_dim]
        returns: [B, out_channels, H, W]
        """
        if self.background_image is None:
            raise ValueError("Background image not set. Please create or load a background image before inference.")

        # concatenate background to ref_img along channels
        B, C, H, W = ref_img.shape
        background = self.background_image.expand(B, -1, -1, -1)  # expand to batch size
        ref_img_and_background = torch.cat([ref_img, background], dim=1)  # [B, C + C_bg, H, W]

        return super().forward(ref_img_and_background, config)


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


    # model = UNetWithFiLM(
    #     config=config, 
    #     accelerator=accelerator,
    # )
    model = UNetWithFiLMAndEnv(
        config=config,
        accelerator=accelerator,
        in_channels=6,  # ref + background
    )
    model.summary(input_size=[(1, 3, *image_shape), (1, len(config["model"]["dimensions_to_learn"]))])

    print(f"shape background_image: {model.background_image.shape}")
    # plot
    import matplotlib.pyplot as plt
    bg_img = model.background_image.squeeze(0).permute(1, 2, 0).cpu().numpy()  # [H, W, C]
    plt.imshow(bg_img)
    plt.axis("off")
    plt.show()

    # dataset and loaders
    config["data"]["num_workers"] = int(os.getenv("NUM_WORKERS", 0)) # set num_workers from .env, default 0
    train_loader, val_loader, test_loader = get_data(config, logger=model.logger, load_reference=True)
    
    
    # train model
    trainer = Trainer(model, config)
    trainer.train_model(train_loader, 
                        val_loader=val_loader if config["training"]["validation"] else None, # early stopping if val_loader is provided
                        early_stopping_enabled=config["training"]["early_stopping"],
                        epochs=config["training"]["epochs"]
                        )
