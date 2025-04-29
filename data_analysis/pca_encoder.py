import torch
import os
from gripper_data import load_image, compute_mean_image, GripperDataset
from sklearn.decomposition import PCA
import PIL
import pickle
import numpy as np
from torchvision import transforms
from torch.utils.data import DataLoader
from tqdm import tqdm
import json

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


class PCAEncoder(torch.nn.Module):
    def __init__(self, model_name, input_dim=200, output_dim=5):
        super(PCAEncoder, self).__init__()
        self.model_name = model_name
        self.model_path = os.path.join(os.path.dirname(__file__), "pca_models", self.model_name)
        os.makedirs(self.model_path, exist_ok=True)
        
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.architecture = torch.nn.Sequential(
            torch.nn.Linear(input_dim, 128),
            torch.nn.ReLU(),
            torch.nn.Linear(128, 64),
            torch.nn.ReLU(),
            torch.nn.Linear(64, 32),
            torch.nn.ReLU(),
            torch.nn.Linear(32, output_dim),
        )
        self.architecture.to(DEVICE)

        filename = os.path.join(os.path.dirname(__file__), "clean_environment_images","mean_mask.jpeg")
        # import mean image, clean environment, if it exists
        if os.path.exists(filename):
            print("Loading existing mean image")
            transform = transforms.Compose([
                transforms.Resize((180, 320)),
                transforms.ToTensor(),  
                transforms.Lambda(lambda x: torch.flatten(x)),
            ])
            self.mean = transform(PIL.Image.fromarray(load_image(filename))).to(DEVICE)
                                    # dtype=torch.float16).flatten()
        else:
            raise FileNotFoundError(f"File {filename} not found.")
        
        filename = os.path.join(os.path.dirname(__file__), "pca_models",'pca_masks_1000.pkl')
        if os.path.exists(filename):
            print("Loading existing PCA model")
            with open(filename, 'rb') as f:
                self.pca = pickle.load(f)
        else:
            raise FileNotFoundError(f"File {filename} not found.")
        self.eigenvectors = torch.tensor(self.pca.components_[:200], 
                                         dtype=torch.float32).to(DEVICE)


    def forward(self, x):
        return self.architecture(x)
    

    def train(self, train_loader, val_loader=None, epochs=1000):
        optimizer = torch.optim.Adam(self.parameters())
        criterion = torch.nn.MSELoss()
        
        train_losses = []
        val_losses = []
        for epoch in tqdm(range(epochs), 
                          desc="Training", 
                          unit="epoch",
                          total=epochs):
            
            # training
            self.architecture.train()
            running_loss = 0.0
            for batch in train_loader:
                images, labels = batch
                images = images.to(DEVICE)
                labels = labels[:,:self.output_dim].to(DEVICE)
                # print(f"Shape: {images.shape}, {self.mean.shape}, {self.eigenvectors.shape}")
                # print(f"Type: {images.dtype}, {self.mean.dtype}, {self.eigenvectors.dtype}")
                # print("min:", images.min(), self.mean.min(), self.eigenvectors.min())
                # print("max:", images.max(), self.mean.max(), self.eigenvectors.max())

                pca_projections = (images - self.mean) @ self.eigenvectors.T
                # print(f"Shape: {pca_projections.shape}, {labels.shape}")
                
                optimizer.zero_grad()
                outputs = self(pca_projections)
                loss = criterion(outputs, labels)
                loss.backward()
                optimizer.step()

                running_loss += loss.item()
            train_loss = np.sqrt(running_loss / len(train_loader)) # RMSE
            print(f'Epoch [{epoch + 1}/{epochs}], RMSE Loss: {train_loss:.4f}')
            print(f"Outputs: {outputs[0].detach().numpy()}, "
                  f"Labels: {labels[0].detach().numpy()}")
            
            train_losses.append(train_loss)
            self.save_learning_curve(train_losses, "train")

            # validation
            if val_loader is not None:
                self.architecture.eval()
                with torch.no_grad():
                    running_loss = 0.0
                    for batch in val_loader:
                        images, labels = batch
                        images = images.to(DEVICE)
                        labels = labels[:,:self.output_dim].to(DEVICE)

                        pca_projections = (images - self.mean) @ self.eigenvectors.T
                        outputs = self(pca_projections)
                        loss = criterion(outputs, labels)
                        running_loss += loss.item()
                    val_loss = np.sqrt(running_loss / len(val_loader))
                    print(f'Validation RMSE Loss: {val_loss:.4f}')
                    val_losses.append(val_loss)
                    self.save_learning_curve(val_losses, "val")

            # checkpoint
            torch.save(self.architecture.state_dict(), 
                    os.path.join(self.model_path, "model.pth"))


    def save_learning_curve(self, losses, curve_name):
        filename = os.path.join(self.model_path, f'{curve_name}_curve.json')
        with open(filename, "w") as file:
            json.dump(losses, file)



if __name__ == "__main__":
    # data
    top_img_shape = np.array([720, 1280])
    resize_factor = 4
    resize_shape = [x for x in map(int,top_img_shape/resize_factor)]
    print(f"Resizing images to {resize_shape}")
    preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize(resize_shape),
            # transforms.ConvertImageDtype(torch.float16),
            transforms.Lambda(lambda x: torch.flatten(x))
            ])
    batch_size = 128 #int(os.getenv("BATCH_SIZE"))
    print(f"Batch size: {batch_size}")
    sessions = np.arange(1,11) # sessions to load
    path = os.getenv("DATASET_PATH") # experiment path
    dataset = GripperDataset(abs_path=path, 
                             sessions=sessions, 
                             transform=preprocess,
                             images_to_load=["Top_masks_processed"])
    # split = [0.9, 0.1] # train, val
    # train_dataset, val_dataset = torch.utils.data.random_split(dataset, split)
    split = [0.8, 0.1, 0.1] # train, val, test
    train_dataset, val_dataset, test_dataset = torch.utils.data.random_split(dataset, split)
    train_loader = DataLoader(train_dataset, 
                            batch_size=batch_size, 
                            shuffle=True)
    val_loader = DataLoader(val_dataset,
                            batch_size=batch_size)

    # model
    input_dim = 200
    output_dim = 2
    model_name = f"pca_encoder_{input_dim}_128_64_32_{output_dim}"
    model = PCAEncoder(model_name, input_dim, output_dim)

    print("Model architecture:")
    print(model.architecture)

    print("Training model...")
    model.train(train_loader, val_loader, epochs=100)
    print("Training complete")