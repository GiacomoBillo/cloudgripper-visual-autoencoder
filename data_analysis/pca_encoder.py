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
from dotenv import load_dotenv
from torch.utils.tensorboard import SummaryWriter

load_dotenv()
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

MODEL_NAME = "debug" #None
VERBOSE = True
SESSIONS = np.arange(1, 3) # sessions to load
HYPERPARAMETERS = { 
    "input_dim": 1000,
    "layers": [128, 64, 32, 3],
    "dropout": 0.2,
    "batch_norm": True,
    "activation": "relu",
    "optimizer": "adamW",
    "learning_rate": 0.0005,
    "weight_decay": 1e-2,
    "batch_size": 128, # 32, 64, 128
    "epochs": 100,
}

def create_name(hyperparameters):
    name = f"pca_encoder_{hyperparameters['input_dim']}"
    for layer in hyperparameters["layers"]:
        name += f"_{layer}"
    name += f"_lr{hyperparameters['learning_rate']}_bs{hyperparameters['batch_size']}"
    if hyperparameters["dropout"] > 0:
        name += f"_dropout{hyperparameters['dropout']}"
    if hyperparameters["activation"] != "relu":
        name += f"_{hyperparameters['activation']}"
    if hyperparameters["optimizer"] != "adamW":
        name += f"_{hyperparameters['optimizer']}"
    if hyperparameters["weight_decay"] > 0:
        name += f"_wd{hyperparameters['weight_decay']}"
    return name


class PCAEncoder(torch.nn.Module):
    def __init__(self, 
                 model_name=None, 
                 hyperparameters=HYPERPARAMETERS,
                 verbose=VERBOSE
                 ):
        super(PCAEncoder, self).__init__()

        # model name and path
        if model_name is not None:
            self.model_name = model_name
        else:
            self.model_name = create_name(HYPERPARAMETERS)
        self.model_path = os.path.join(os.path.dirname(__file__), "pca_models")
        os.makedirs(self.model_path, exist_ok=True)
        
        # hyperparameters
        self.input_dim = hyperparameters["input_dim"]
        self.layers = hyperparameters["layers"]
        self.output_dim = hyperparameters["layers"][-1]
        self.lr = hyperparameters["learning_rate"]
        self.batch_size = hyperparameters["batch_size"]
        self.dropout = hyperparameters["dropout"]
        self.batch_norm = hyperparameters["batch_norm"]
        self.weight_decay = hyperparameters["weight_decay"]

        previous_size = self.input_dim
        layers = []
        for i, layer_size in enumerate(self.layers):
            layers.append(torch.nn.Linear(previous_size, layer_size))
            if i != len(self.layers) - 1: # linear activation for the last layer
                if self.batch_norm:
                    layers.append(torch.nn.BatchNorm1d(layer_size))
                layers.append(torch.nn.ReLU())
                if self.dropout > 0:
                    layers.append(torch.nn.Dropout(self.dropout))
            previous_size = layer_size
        self.architecture = torch.nn.Sequential(*layers)
        self.architecture.to(DEVICE)
        # self.eval()

        # learning curve writer
        self.writer = SummaryWriter(log_dir=os.path.join(self.model_path)) # each model has its own folder
        # self.writer = SummaryWriter(log_dir=os.path.join(os.path.dirname(__file__), "runs", self.model_name)) # separated folder only for summary_writer
        self.writer.add_text("Model name", self.model_name)
        self.writer.add_text("Hyperparameters", json.dumps(hyperparameters))
        # self.writer.add_graph(self, verbose=VERBOSE)
        self.writer.add_text("Model architecture", str(self.architecture))
        self.writer.flush()

        # filename = os.path.join(os.path.dirname(__file__), "clean_environment_images","mean_mask.jpeg")
        # # import mean image, clean environment, if it exists
        # if os.path.exists(filename):
        #     print("Loading existing mean image")
        #     transform = transforms.Compose([
        #         transforms.Resize((180, 320)),
        #         transforms.ToTensor(),  
        #         transforms.Lambda(lambda x: torch.flatten(x)),
        #     ])
        #     self.mean = transform(PIL.Image.fromarray(load_image(filename))).to(DEVICE)
        #                             # dtype=torch.float16).flatten()
        # else:
        #     raise FileNotFoundError(f"File {filename} not found.")
        
        filename = os.path.join(os.path.dirname(__file__), "pca_models",'pca_masks_1000.pkl')
        if os.path.exists(filename):
            # print("Loading existing PCA model")
            with open(filename, 'rb') as f:
                self.pca = pickle.load(f)
        else:
            raise FileNotFoundError(f"File {filename} not found.")
        # self.eigenvectors = torch.tensor(self.pca.components_[:200], 
        #                                  dtype=torch.float32).to(DEVICE)

        if verbose:
            print(f"Model name: {self.model_name}")
            print(f"Model path: {self.model_path}")
            print(f"Input dimension: {self.input_dim}")
            print(f"Layers: {self.layers}")
            print(f"Output dimension: {self.output_dim}")
            print(f"Learning rate: {self.lr}")
            print(f"Batch size: {self.batch_size}")
            print(f"Dropout: {self.dropout}")
            print(f"Batch norm: {self.batch_norm}")
            print(f"Weight decay: {self.weight_decay}")

            print(f"\nModel architecture: {self.architecture}")


    def forward(self, x):
        return self.architecture(x)
    

    def train(self, 
              train_loader: DataLoader, 
              val_loader: DataLoader = None, 
              epochs=HYPERPARAMETERS["epochs"], 
              lr=HYPERPARAMETERS["learning_rate"],
              weight_decay=HYPERPARAMETERS["weight_decay"],
              verbose=VERBOSE
              ):
        # optimizer = torch.optim.Adam(self.parameters(), lr=lr)
        optimizer = torch.optim.AdamW(self.parameters(), lr=lr, weight_decay=weight_decay)
        criterion = torch.nn.MSELoss()

        if verbose:
            print(f"Training model {self.model_name}...")
            print(f"Training on {DEVICE}")
        self.writer.add_text("Training", f"Training with train dataset of length {len(train_loader)} batches of size {self.batch_size}")
        if val_loader is not None:
            self.writer.add_text("Validation", f"Validation with val dataset of length {len(val_loader)} batches of size {self.batch_size}")

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

                # pca_projections = images @ self.eigenvectors.T
                pca_projections = self.pca.transform(images)[:,:self.input_dim]
                pca_projections = torch.tensor(pca_projections, device=DEVICE, dtype=torch.float32)
                # print(f"Shape: {pca_projections.shape}, {labels.shape}")
                
                optimizer.zero_grad()
                outputs = self(pca_projections)
                loss = criterion(outputs, labels)
                loss.backward()
                optimizer.step()

                running_loss += loss.item()
            train_loss = np.sqrt(running_loss / len(train_loader)) # RMSE
            if verbose:
                print(f'\nEpoch [{epoch + 1}/{epochs}], RMSE Loss: {train_loss:.4f}')
                print(f"Outputs: {outputs[0].detach().cpu().numpy()}, "
                    f"Labels: {labels[0].detach().cpu().numpy()}")
            self.writer.add_scalar("Loss/train", train_loss, epoch)
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

                        # pca_projections = images @ self.eigenvectors.T
                        pca_projections = self.pca.transform(images)[:,:self.input_dim]
                        pca_projections = torch.tensor(pca_projections, device=DEVICE, dtype=torch.float32)
                        outputs = self(pca_projections)
                        loss = criterion(outputs, labels)
                        running_loss += loss.item()
                    val_loss = np.sqrt(running_loss / len(val_loader))
                    if verbose:
                        print(f'Validation RMSE Loss: {val_loss:.4f}')
                    self.writer.add_scalar("Loss/val", val_loss, epoch)
                    val_losses.append(val_loss)
                    self.save_learning_curve(val_losses, "val")
                self.writer.add_scalars("Learning_Curves", {"train": train_loss, "val": val_loss}, epoch)

            # checkpoint
            torch.save(self.architecture.state_dict(), 
                    os.path.join(self.model_path, "model.pth"))
        
        self.writer.flush()
        self.writer.close()


    def save_learning_curve(self, losses, curve_name):
        filename = os.path.join(self.model_path, f'{curve_name}_curve.json')
        with open(filename, "w") as file:
            json.dump(losses, file)


    def load_model(self):
        self.architecture.load_state_dict(
            torch.load(os.path.join(self.model_path, "model.pth"), 
            map_location=DEVICE))



if __name__ == "__main__":
    print("Hyperparameters:")
    for key, value in HYPERPARAMETERS.items():
        print(f"{key}: {value}")
    # data
    top_img_shape = np.array([720, 1280])
    resize_factor = 4
    resize_shape = [x for x in map(int,top_img_shape/resize_factor)]
    # print(f"Resizing images to {resize_shape}")
    preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize(resize_shape),
            # transforms.ConvertImageDtype(torch.float16),
            transforms.Lambda(lambda x: torch.flatten(x))
            ])
    batch_size = HYPERPARAMETERS["batch_size"] #int(os.getenv("BATCH_SIZE"))
    sessions = SESSIONS #np.arange(1,3) # sessions to load
    path = os.getenv("DATASET_PATH") # experiment path
    dataset = GripperDataset(abs_path=path, 
                             sessions=sessions, 
                             transform=preprocess,
                             images_to_load=["Top_masks_processed"])
    # split = [0.9, 0.1] # train, val
    # train_dataset, val_dataset = torch.utils.data.random_split(dataset, split)
    split = [0.8, 0.1, 0.1] # train, val, test
    torch.manual_seed(666) # for reproducibility
    train_dataset, val_dataset, test_dataset = torch.utils.data.random_split(dataset, split)
    train_loader = DataLoader(train_dataset, 
                            batch_size=batch_size, 
                            shuffle=True)
    val_loader = DataLoader(val_dataset,
                            batch_size=batch_size)

    # model
    model = PCAEncoder(model_name=MODEL_NAME, verbose=False)

    print("\nModel architecture:")
    print(model.architecture)

    print("Training model...")
    model.train(train_loader, val_loader, verbose=VERBOSE)
    print("Training complete")