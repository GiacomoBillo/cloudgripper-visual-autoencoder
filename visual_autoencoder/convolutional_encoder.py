import torch
import torch.nn as nn
from torchinfo import summary
import os
from gripper_data import GripperDataset
import numpy as np
from torchvision import transforms
from torch.utils.data import DataLoader
from tqdm import tqdm
import json
from dotenv import load_dotenv
from torch.utils.tensorboard import SummaryWriter


load_dotenv()
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

DIM_TO_LEARN = [0,2] # x,z
MODEL_NAME = "Debug" # None
VERBOSE = True
# SESSIONS = np.arange(1, 3) # sessions to load
HYPERPARAMETERS = { 
    "batch_size": 32, # 32, 64, 128
}

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
                 model_name=MODEL_NAME, 
                 in_channels=3):
        super(ConvolutionalEncoder, self).__init__()

        # model name and path
        if model_name is not None:
            self.model_name = model_name
        self.model_path = os.path.join(os.path.dirname(__file__), "convolutional_encoder", self.model_name)
        os.makedirs(self.model_path, exist_ok=True)

        # architecture
        self.encoder = torch.nn.Sequential( # input [3, 45, 80]
            EncoderBlock(in_channels=in_channels, out_channels=8),   # [8, 22, 40]
            EncoderBlock(in_channels=8, out_channels=16),            # [16, 11, 20]
            EncoderBlock(in_channels=16, out_channels=32),           # [32, 5, 10]
            EncoderBlock(in_channels=32, out_channels=64, pooling=False),           # [64, 5, 10] no pooling

            torch.nn.Flatten(),
            
            # fully connected layer to 5D latent space
            torch.nn.Linear(in_features=64*5*10, out_features=2), # flatten and reduce dimension
            torch.nn.Sigmoid() # to keep outputs between 0 and 1
        )
        # print(self.encoder)

    def forward(self, x):
        return self.encoder(x)
    
    def _train_step(self, batch):
        images, labels = batch
        images = images.to(DEVICE)
        labels = labels.to(DEVICE)
        
        self.optimizer.zero_grad()
        outputs = self(images)
        # print("\noutputs:\n", outputs)
        # print("\nlabels:\n", labels)
        loss = self.criterion(outputs, labels[:,DIM_TO_LEARN])
        loss.backward()
        self.optimizer.step()
        
        return loss
    
    def _val_step(self, batch, verbose=False):
        images, labels = batch
        images = images.to(DEVICE)
        labels = labels.to(DEVICE)
        
        with torch.no_grad():
            outputs = self(images)
            loss = self.criterion(outputs, labels[:,DIM_TO_LEARN]) 

        if verbose:
            print(f"\nPredictions: x={outputs[0,0]:.2f}, z={outputs[0,1]:.2f}")
            print(f"Ground truth: x={labels[0,0]:.2f}, z={labels[0,2]:.2f}")
        
        return loss
    
    def train_model(self, train_loader, val_loader, epochs=20, learning_rate=1e-3, verbose=True, patience=10):
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
            for i, batch in enumerate(train_loader):
                loss = self._train_step(batch)
                running_loss += loss.item() 
                if i>len(train_loader)//4 :
                    break
            train_loss = np.sqrt(running_loss / len(train_loader)) # from MSE to RMSE

            # validation
            self.eval()
            running_loss = 0.0
            for i, batch in enumerate(val_loader):
                loss = self._val_step(batch, verbose=i==len(val_loader)-1)
                running_loss += loss.item()
                if i>len(val_loader)//4 :
                    break
            val_loss = np.sqrt(running_loss / len(val_loader)) # from MSE to RMSE

            # print one sample prediction vs ground truth
            if verbose:
                image, label = val_loader.dataset[np.random.randint(len(val_loader.dataset))]
                image = image.unsqueeze(0)  # add batch dim
                label = label.unsqueeze(0)  # add batch dim
                loss = self._val_step((image, label), verbose=True)
                print(f"\nTraining RMSE Loss: {train_loss:.4f}")
                print(f'Validation RMSE Loss: {val_loss:.4f}')
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
            torch.save(self.state_dict(), 
                os.path.join(self.model_path, "model.pth"))
        
        writer.flush()
        writer.close()


    def save_learning_curve(self, losses, curve_name):
        filename = os.path.join(self.model_path, f'{curve_name}_curve.json')
        with open(filename, "w") as file:
            json.dump(losses, file)


    def load_model(self):
        self.load_state_dict(
            torch.load(os.path.join(self.model_path, "model.pth"), 
            map_location=DEVICE))
        

if __name__ == "__main__":
    model = ConvolutionalEncoder().to(DEVICE)
    # print(model)
    summary(model, input_size=(1, 3, 45, 80))

    # dimension example with random input
    # x = torch.randn((1, 3, 720//8, 1280//8)).to(DEVICE)
    # print("Input shape:", x.shape)
    # output = model(x)
    # print("Output shape:", output.shape)  # should be [1, 5]

    # dataset
    top_img_shape = np.array([720, 1280])
    resize_factor = 16
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
    torch.manual_seed(6) # for reproducibility
    # split = [0.9, 0.1] # train, val
    # train_dataset, val_dataset = torch.utils.data.random_split(dataset, split)
    split = [8000, 2000, 90000] # train, val, test
    train_dataset, val_dataset, test_dataset = torch.utils.data.random_split(dataset, split)
    print(f"Total dataset size: {len(dataset)}")
    print(f"Len train dataset: {len(train_dataset)}, Len val dataset: {len(val_dataset)}")
    train_loader = DataLoader(train_dataset, 
                            batch_size=batch_size, 
                            shuffle=True)
    val_loader = DataLoader(val_dataset,
                            batch_size=batch_size)
    
    # plot one downscaled image
    # import matplotlib.pyplot as plt
    # image_np = dataset[0][0].permute(1, 2, 0).numpy()
    # plt.imshow(image_np)
    # plt.axis('off')  # Hide axis
    # plt.show()

    print("Training model...")
    model.train_model(train_loader, val_loader, verbose=VERBOSE)
    print("Training complete")