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


class PCAEncoder(torch.nn.Module):
    def __init__(self, input_dim=200, output_dim=5):
        super(PCAEncoder, self).__init__()
        self.model_path = os.path.join(os.path.dirname(__file__), "pca_models", "pca_encoder.pth")
        self.input_dim = input_dim
        self.output_dim = output_dim
        self.architecture = torch.nn.Sequential(
            torch.nn.Linear(input_dim, 100),
            torch.nn.ReLU(),
            torch.nn.Linear(100, 50),
            torch.nn.ReLU(),
            torch.nn.Linear(50, output_dim),
        )

        filename = os.path.join(os.path.dirname(__file__), "clean_environment_images","mean_mask.jpeg")
        # import mean image, clean environment, if it exists
        if os.path.exists(filename):
            print("Loading existing mean image")
            transform = transforms.Compose([
                transforms.Resize((180, 320)),
                transforms.ToTensor(),  
                transforms.Lambda(lambda x: torch.flatten(x)),
            ])
            self.mean = transform(PIL.Image.fromarray(load_image(filename)))
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
                                         dtype=torch.float32)


    def forward(self, x):
        return self.architecture(x)
    

    def train(self, train_loader, epochs=1000):
        self.architecture.train()

        optimizer = torch.optim.Adam(self.parameters())
        criterion = torch.nn.MSELoss()
        
        for epoch in tqdm(range(epochs), 
                          desc="Training", 
                          unit="epoch",
                          total=epochs):
            running_loss = 0.0

            for batch in train_loader:
                images, labels = batch
                labels = labels[:,:2]
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
            epoch_loss = running_loss / len(train_loader)
            print(f'Epoch [{epoch + 1}/{epochs}], RMSE Loss: {np.sqrt(epoch_loss):.4f}')
            print(f"Outputs: {outputs[0].detach().numpy()}, "
                  f"Labels: {labels[0].detach().numpy()}")

        torch.save(self.architecture.state_dict(), self.model_path)



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
    sessions = np.arange(1,3) # sessions to load
    path = os.getenv("DATASET_PATH") # experiment path
    dataset = GripperDataset(abs_path=path, 
                             sessions=sessions, 
                             transform=preprocess,
                             images_to_load=["Top_masks_processed"])
    dataloader = DataLoader(dataset, 
                            batch_size=batch_size, 
                            shuffle=True)

    # model    
    input_dim = 200
    output_dim = 2
    model = PCAEncoder(input_dim, output_dim)

    print("Model architecture:")
    print(model.architecture)

    print("Training model...")
    model.train(dataloader, epochs=300)
    print("Training complete")