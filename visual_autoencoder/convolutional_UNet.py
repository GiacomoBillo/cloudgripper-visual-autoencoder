import torch
import torch.nn as nn
from torchinfo import summary
import os
from gripper_data_ref import GripperDataset
import numpy as np
from torchvision import transforms
from torch.utils.data import DataLoader
from tqdm import tqdm
import json
from dotenv import load_dotenv
from torch.utils.tensorboard import SummaryWriter
import torch.nn.functional as F
from typing import Optional, Tuple, List, Dict

load_dotenv()
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

DIM_TO_LEARN = [0,2] # x,z
MODEL_NAME = "unet_film_ref_delta_xz_1ref" 
VERBOSE = True
# SESSIONS = np.arange(1, 3) # sessions to load
HYPERPARAMETERS = { 
    "batch_size": 32, # 32, 64, 128
}

""" 
initial top image size: [3, 720, 1280] [batch_size, channels, height, width]
downscale /8 to [3, 90, 160]
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


class UNetWithFiLM(nn.Module):
    """
    UNet where conditioning vector is mapped via ConfigMLP to FiLM parameters that are applied
    at every ConvBlock (encoder and decoder).
    """
    def __init__(self,
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
        super().__init__()
        self.model_path = os.path.join("models", MODEL_NAME)
        if not os.path.exists(self.model_path):
            os.makedirs(self.model_path)
        self.in_channels = in_channels
        self.config_dim = config_dim
        self.base_channels = base_channels
        self.num_downs = num_downs

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
    
    def _train_step(self, batch,delta=False):
        images, labels, reference_images, reference_labels  = batch
        if delta:
            labels = labels - reference_labels
        images = images.to(DEVICE)
        labels = labels.to(DEVICE)
        reference_images = reference_images.to(DEVICE)

        
        self.optimizer.zero_grad()
        
        outputs = self(reference_images,labels)
        # print("\noutputs:\n", outputs)
        # print("\nlabels:\n", labels)
        loss = self.criterion(outputs, images)
        loss.backward()
        self.optimizer.step()
        
        return loss
    
    def _val_step(self, batch,delta = False,verbose=False):
        images, labels, reference_images, reference_labels  = batch
        if delta:
            labels = labels - reference_labels
        images = images.to(DEVICE)
        labels = labels.to(DEVICE)
        reference_images = reference_images.to(DEVICE)
        with torch.no_grad():
            outputs = self(reference_images,labels)
            loss = self.criterion(outputs, images) 
            
        return loss
    
    def train_model(self, train_loader, val_loader, epochs=100, learning_rate=1e-3, verbose=True, patience=100):
        self.to(DEVICE)
        self.optimizer = torch.optim.Adam(self.parameters(), lr=learning_rate)
        self.criterion = torch.nn.MSELoss()

        writer = SummaryWriter() # for tensorboard

        train_losses = []
        val_losses = []
        count_patience = 0
        best_val_loss = float("inf")
        for epoch in tqdm(range(epochs), 
                          desc="Training", 
                          unit="epoch",
                          total=epochs):
            
            self.train()
            running_loss = 0.0
            running_count = 0
            for i, batch in enumerate(train_loader):
                loss = self._train_step(batch)
                running_loss += loss.item()*len(batch[0])
                running_count += len(batch[0])
                # if i>len(train_loader)//2 :
                #     break
            train_loss = np.sqrt(running_loss / running_count) 

            # validation
            self.eval()
            running_loss = 0.0
            for i, batch in enumerate(val_loader):
                loss = self._val_step(batch, verbose=i==len(val_loader)-1)
                running_loss += loss.item()
                # if i>len(val_loader)//2 :
                #     break
            val_loss = np.sqrt(running_loss / (len(val_loader)//2)) # from MSE to RMSE

            # print one sample prediction vs ground truth
            # if verbose:
                # image, label, reference_image, reference_label = val_loader.dataset[np.random.randint(len(val_loader.dataset))]
                # image = image.unsqueeze(0)  # add batch dim
                # label = label.unsqueeze(0)  # add batch dim
                # loss = self._val_step((image, label), verbose=True)
                # print(f"\nTraining RMSE Loss: {train_loss:.4f}")
                # print(f'Validation RMSE Loss: {val_loss:.4f}')
                # # from visualizer import show_tensor_image
                # # show_tensor_image(image[0].cpu(), title="Validation Image", label=(label[0,0].item(), label[0,2].item()))
            writer.add_scalar("Loss/train", train_loss, epoch)
            train_losses.append(train_loss)
            self.save_learning_curve(train_losses, "train")
            writer.add_scalar("Loss/val", val_loss, epoch)
            val_losses.append(val_loss)
            self.save_learning_curve(val_losses, "val")
            writer.add_scalars("Learning_Curves", {"train": train_loss, "val": val_loss}, epoch)

            # early stopping
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                count_patience = 0
            else:
                count_patience += 1
                print(f"Patience count: {count_patience}/{patience}")
                if count_patience >= patience:
                    writer.add_text("Early stopping", f"Early stopping at epoch {epoch + 1}")
                    if verbose:
                        print(f"Early stopping at epoch {epoch + 1}")
                    break

            # checkpoint
            if (epoch + 1) % 20 == 0:
                torch.save(self.state_dict(), 
                    os.path.join(self.model_path, f"model_epoch{epoch}.pth"))
        
        writer.flush()
        writer.close()


    def save_learning_curve(self, losses, curve_name):
        filename = os.path.join(self.model_path, f'{curve_name}_curve.json')
        with open(filename, "w") as file:
            json.dump(losses, file)


    def load_model(self):
        self.load_state_dict(
            torch.load(os.path.join(self.model_path, "model_epoch99.pth"), 
            map_location=DEVICE,weights_only=True))
        

if __name__ == "__main__":
    train = True
    test = True
    rollout = True

    model = UNetWithFiLM(base_channels=16).to(DEVICE)

    # dataset
    top_img_shape = np.array([720, 1280])
    resize_factor = 8
    resize_shape = [x for x in map(int,top_img_shape//resize_factor)]
    print(f"Resizing images from {top_img_shape} to {resize_shape}")
    preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize(resize_shape),
            ])
    batch_size = HYPERPARAMETERS["batch_size"] #int(os.getenv("BATCH_SIZE"))
    # sessions = np.arange(1,21) # sessions to load
    
    path = os.getenv("DATASET_PATH") # experiment path
    dataset = GripperDataset(abs_path=path, 
                            #  sessions=sessions, 
                             transform=preprocess,
                             images_to_load=["Images"])
    # dataset.set_references([a for a in np.arange(len(dataset)) if a%1000==0]) # set every 10th sample as reference
    dataset.set_references() # chooses automatically representative references, currently set to 1 reference
    torch.manual_seed(6) # for reproducibility
    n = len(dataset)
    split = [int(n*0.8), int(n*0.1)]
    split.append(n - sum(split))
    train_dataset, val_dataset, test_dataset = torch.utils.data.random_split(dataset, split)
    print(f"Total dataset size: {len(dataset)}")
    print(f"Len train dataset: {len(train_dataset)}, Len val dataset: {len(val_dataset)}")
    train_loader = DataLoader(
        train_dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=4,       # or more if you have CPU cores free
        pin_memory=True,     # speeds up GPU transfer
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=batch_size,
        num_workers=4,
        pin_memory=True,
    )
    

    if train:
        print("Training model...")
        model.train_model(train_loader, val_loader, epochs=200,verbose=VERBOSE)
        print("Training complete")
    
    if test:
        print("Testing model...")
        model.load_model()
        model.criterion = torch.nn.MSELoss()
        model.eval()
        test_loader = DataLoader(
            test_dataset,
            batch_size=1,
            num_workers=4,
            pin_memory=True,
        )
        running_loss = 0.0
        for i, batch in enumerate(test_loader):
            loss = model._val_step(batch, verbose=False)
            running_loss += loss.item()
        test_loss = np.sqrt(running_loss / (len(test_loader))) # from MSE to RMSE
        print(f"Test RMSE Loss: {test_loss:.4f}")

    if rollout:
        import matplotlib.pyplot as plt
        from torchvision.transforms.functional import to_pil_image

        print("Rollout model...")
        model.load_model()
        model.criterion = torch.nn.MSELoss()
        model.eval()
        test_loader = DataLoader(
            test_dataset,
            batch_size=1,
            shuffle=False,
            num_workers=1,
            pin_memory=True,
        )
        for i, batch in enumerate(test_loader):
            images, labels, reference_images, reference_labels  = batch
            images = images.to(DEVICE)
            labels = labels.to(DEVICE)
            reference_images = reference_images.to(DEVICE)

            with torch.no_grad():
                outputs = model(reference_images,labels)
            
            # visualize
            input_img = to_pil_image(reference_images[0].cpu())
            gt_img = to_pil_image(images[0].cpu())
            output_img = to_pil_image(outputs[0].cpu().clamp(0,1))

            fig, axs = plt.subplots(1,3, figsize=(12,4))
            axs[0].imshow(input_img)
            axs[0].set_title("Input (Reference)")
            axs[0].axis('off')
            axs[1].imshow(gt_img)
            axs[1].set_title("Ground Truth")
            axs[1].axis('off')
            axs[2].imshow(output_img)
            axs[2].set_title("Model Output")
            axs[2].axis('off')
            plt.show()

            if i>=5:
                break
