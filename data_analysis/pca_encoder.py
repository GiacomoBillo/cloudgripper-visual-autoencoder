import torch
import os
from gripper_data import GripperDataset
from sklearn.decomposition import PCA
import pickle
import numpy as np
from torchvision import transforms
from torch.utils.data import DataLoader
from tqdm import tqdm
import json
from dotenv import load_dotenv
from torch.utils.tensorboard import SummaryWriter
# from fc_model import normalize_principal_components
import torch.nn.utils.prune as prune

load_dotenv()
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

MODEL_NAME = None
VERBOSE = True
SESSIONS = np.arange(1, 3) # sessions to load
HYPERPARAMETERS = { 
    "input_dim": 200,
    "layers": [32, 8, 1],
    "dropout": 0.3,
    "input_norm": "False", # "False", "Sample-wise", "MinMax"
    "normalization": "False", # "BatchNorm", "LayerNorm", False
    # Batch Normalization -> normalization across features
    # Layer Normalization -> normalization across samples
    "activation": "gelu",
    "optimizer": "adamW",
    "learning_rate": 0.001,
    "weight_decay": 0.01,
    "batch_size": 32, # 32, 64, 128
    "epochs": 100,
    "network_pruning": False, # "global", "local"
}

def create_name(hyperparameters):
    name = f"pca_encoder_{hyperparameters['input_dim']}"
    for layer in hyperparameters["layers"]:
        name += f"_{layer}"
    name += f"_lr{hyperparameters['learning_rate']}_bs{hyperparameters['batch_size']}"
    if hyperparameters["normalization"] == "BatchNorm" or hyperparameters["normalization"] == "LayerNorm":
        name += f"_{hyperparameters['normalization']}"
    if hyperparameters["dropout"] > 0:
        name += f"_dropout{hyperparameters['dropout']}"
    if hyperparameters["activation"] != "relu":
        name += f"_{hyperparameters['activation']}"
    if hyperparameters["optimizer"] != "adamW":
        name += f"_{hyperparameters['optimizer']}"
    if hyperparameters["weight_decay"] > 0:
        name += f"_wd{hyperparameters['weight_decay']}"
    if hyperparameters["network_pruning"]:
        name += f"_pruning_{hyperparameters['network_pruning']}"
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
            self.model_name = create_name(hyperparameters)
        self.model_path = os.path.join(os.path.dirname(__file__), "pca_encoder", self.model_name)
        os.makedirs(self.model_path, exist_ok=True)
        
        # hyperparameters
        self.input_dim = hyperparameters["input_dim"]
        self.layers = hyperparameters["layers"]
        self.output_dim = hyperparameters["layers"][-1]
        self.lr = hyperparameters["learning_rate"]
        self.batch_size = hyperparameters["batch_size"]
        self.dropout = hyperparameters["dropout"]
        self.normalization = hyperparameters["normalization"]
        self.weight_decay = hyperparameters["weight_decay"]
        self.input_norm = hyperparameters["input_norm"]
        self.activation = hyperparameters["activation"]
        self.network_pruning = hyperparameters["network_pruning"]

        # self.batch_norm_input = torch.nn.BatchNorm1d(self.input_dim)
        # self.batch_norm_input.to(DEVICE)

        previous_size = self.input_dim
        layers = []
        for i, layer_size in enumerate(self.layers):
            if i == 0: # to normalize the input
                if self.normalization == "BatchNorm":
                    layers.append(torch.nn.BatchNorm1d(self.input_dim))
                elif self.normalization == "LayerNorm":
                    layers.append(torch.nn.LayerNorm(self.input_dim))
            layers.append(torch.nn.Linear(previous_size, layer_size))
            if i != len(self.layers) - 1: # for the hidden layer
                # normalization
                if self.normalization == "BatchNorm":
                    layers.append(torch.nn.BatchNorm1d(layer_size))
                elif self.normalization == "LayerNorm":
                    layers.append(torch.nn.LayerNorm(layer_size))

                # activation for hidden layers
                if self.activation == "relu":
                    layers.append(torch.nn.ReLU())
                elif self.activation == "gelu":
                    layers.append(torch.nn.GELU())
                elif self.activation == "elu":
                    layers.append(torch.nn.ELU())
                
                # dropout
                if self.dropout > 0:
                    layers.append(torch.nn.Dropout(self.dropout))
            else:
                layers.append(torch.nn.Sigmoid())
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
            print(f"Batch norm: {self.normalization}")
            print(f"Weight decay: {self.weight_decay}")

            print(f"\nModel architecture: {self.architecture}")
            total_params = sum(p.numel() for p in self.parameters())
            print("Number of parameters:", total_params)
            print()


    def forward(self, x):
        return self.architecture(x)
    

    def train_step(self, batch):
        images, labels = batch
        images = images.to(DEVICE)
        labels = labels[:,:self.output_dim].to(DEVICE)
        # print(f"Shape: {images.shape}, {self.mean.shape}, {self.eigenvectors.shape}")
        # print(f"Type: {images.dtype}, {self.mean.dtype}, {self.eigenvectors.dtype}")
        # print("min:", images.min(), self.mean.min(), self.eigenvectors.min())
        # print("max:", images.max(), self.mean.max(), self.eigenvectors.max())

        pca_projections = self.pca.transform(images.cpu().numpy())[:,:self.input_dim]
        # print(f"Shape: {pca_projections.shape}, {labels.shape}")

        if self.input_norm == "False":
            pca_projections = pca_projections / self.pca.explained_variance_ratio_[0] / 100
            norm_pca_projections = torch.tensor(pca_projections, device=DEVICE, dtype=torch.float32)
        
        else: 

            # standard normalization (mean, var) each component
            # norm_pca_projections = self.normalizer.transform(pca_projections)
            # pca_projections = self.batch_norm_input(pca_projections)

            if self.input_norm == "Sample-wise":
                # normalization sample-wise
                norm_pca_projections = pca_projections / np.linalg.norm(pca_projections, axis=1, keepdims=True)

            elif self.input_norm == "MinMax":
                min = np.min(pca_projections)
                max = np.max(pca_projections)
                # print(min, max)
                norm_pca_projections = (pca_projections - min) / (max - min)
                norm_pca_projections = norm_pca_projections * 2 - 1 # scale to [-1, 1]
                # print(norm_pca_projections)
            norm_pca_projections = torch.tensor(norm_pca_projections, device=DEVICE, dtype=torch.float32)

        self.optimizer.zero_grad()
        outputs = self(norm_pca_projections)
        loss = self.criterion(outputs, labels)
        loss.backward()
        self.optimizer.step()
        
        return loss.item()


    def validation_step(self, batch):
        images, labels = batch
        images = images.to(DEVICE)
        labels = labels[:,:self.output_dim].to(DEVICE)

        # pca_projections = images @ self.eigenvectors.T
        pca_projections = self.pca.transform(images.cpu().numpy())[:,:self.input_dim]

        if self.input_norm == "False":
            pca_projections = pca_projections / self.pca.explained_variance_ratio_[0] / 100
            norm_pca_projections = torch.tensor(pca_projections, device=DEVICE, dtype=torch.float32)

        else:
            # standard normalization (mean, var) each component
            # norm_pca_projections = self.normalizer.transform(pca_projections)
            # pca_projections = self.batch_norm_input(pca_projections)

            if self.input_norm == "Sample-wise":
                # normalization sample-wise
                norm_pca_projections = pca_projections / np.linalg.norm(pca_projections, axis=1, keepdims=True)

            elif self.input_norm == "MinMax":
                min = np.min(pca_projections)
                max = np.max(pca_projections)
                # print(min, max)
                norm_pca_projections = (pca_projections - min) / (max - min)
                norm_pca_projections = norm_pca_projections * 2 - 1 # scale to [-1, 1]
                # print(norm_pca_projections)
            norm_pca_projections = torch.tensor(norm_pca_projections, device=DEVICE, dtype=torch.float32)

        outputs = self(norm_pca_projections)
        loss = self.criterion(outputs, labels)

        return loss.item()


    def train_model(self, 
                    train_loader: DataLoader, 
                    val_loader: DataLoader = None, 
                    epochs=HYPERPARAMETERS["epochs"], 
                    lr=None,
                    weight_decay=None,
                    verbose=False,
                    patience=10,
                    loss_every_n_batches=None,
                    ):
        # self.normalizer = normalize_principal_components(pca = self.pca, 
        #                                                  train_loader = train_loader, 
        #                                                  num_components = self.input_dim)

        if lr is None:
            lr = self.lr
        if weight_decay is None:
            weight_decay = self.weight_decay
        # optimizer = torch.optim.Adam(self.parameters(), lr=lr)
        self.optimizer = torch.optim.AdamW(self.parameters(), lr=lr, weight_decay=weight_decay)
        self.criterion = torch.nn.MSELoss()

        if verbose:
            print(f"Training model {self.model_name}...")
            print(f"Training on {DEVICE}")
        self.writer.add_text("Training", f"Training with train dataset of length {len(train_loader)} batches of size {self.batch_size}")
        if val_loader is not None:
            self.writer.add_text("Validation", f"Validation with val dataset of length {len(val_loader)} batches of size {self.batch_size}")

        train_losses = []
        val_losses = []
        count_patience = 0
        best_val_loss = float("inf")
        for epoch in tqdm(range(epochs), 
                          desc="Training", 
                          unit="epoch",
                          total=epochs):

            # network pruning for regularization
            if self.network_pruning and (0<epoch<11 and epoch % 2 == 0):
                print("\nPruning model...")
                parameters_to_prune = []

                # Global unstructured pruning
                for module in self.architecture:
                    if isinstance(module, torch.nn.Linear):
                        parameters_to_prune.append((module, 'weight'))
                        # if module.bias is not None:
                        #     parameters_to_prune.append((module, 'bias'))
                        if self.network_pruning == "local":
                            prune.ln_structured(
                                module,
                                name='weight',
                                amount=0.1,
                                n=2, # L2 norm
                                dim=0, # prune entire rows
                            )
                            # break

                # Apply global unstructured pruning
                if self.network_pruning == "global":
                    prune.global_unstructured(
                        parameters_to_prune,
                        pruning_method=prune.L1Unstructured,
                        amount=0.1,
                    )
                total_params = sum(p.numel() for p in self.parameters())
                print("Number of parameters:", total_params)
            
            # training
            self.architecture.train()
            running_loss = 0.0
            for i, batch in enumerate(train_loader):
                loss = self.train_step(batch)
                running_loss += loss
                # monitor batch loss
                if loss_every_n_batches is not None and (i+1) % loss_every_n_batches == 0:
                    train_loss = np.sqrt(running_loss / (i+1)) # RMSE
                    self.writer.add_scalar("BatchLoss/train", train_loss, epoch * loss_every_n_batches)
                    if verbose:
                        print(f"\nBatch {epoch * loss_every_n_batches}, Loss: {train_loss:.4f}")
                    break
            train_loss = np.sqrt(running_loss / len(train_loader)) # RMSE
            
                # print(f"Outputs: {outputs[0].detach().cpu().numpy()}, "
                #     f"Labels: {labels[0].detach().cpu().numpy()}")
                # print(f"Projections: {norm_pca_projections[0][:10].detach().cpu().numpy()}")
            if loss_every_n_batches is None:
                self.writer.add_scalar("Loss/train", train_loss, epoch)
                if verbose:
                    print(f'\nEpoch [{epoch + 1}/{epochs}], RMSE Loss: {train_loss:.4f}')
            train_losses.append(train_loss)
            self.save_learning_curve(train_losses, "train")

            # validation
            if val_loader is not None:
                self.architecture.eval()
                with torch.no_grad():
                    running_loss = 0.0
                    for batch in val_loader:
                        loss = self.validation_step(batch)
                        # self.writer.add_scalar("BatchLoss/val", loss) # monitor batch loss
                        running_loss += loss
                    val_loss = np.sqrt(running_loss / len(val_loader))
                    if verbose:
                        print(f'Validation RMSE Loss: {val_loss:.4f}')
                    self.writer.add_scalar("Loss/val", val_loss, epoch)
                    val_losses.append(val_loss)
                    self.save_learning_curve(val_losses, "val")

                    # early stopping
                    if val_loss < best_val_loss:
                        best_val_loss = val_loss
                        count_patience = 0
                    else:
                        count_patience += 1
                        if count_patience >= patience:
                            self.writer.add_text("Early stopping", f"Early stopping at epoch {epoch + 1}")
                            if verbose:
                                print(f"Early stopping at epoch {epoch + 1}")
                            break
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
    # print("Hyperparameters:")
    # for key, value in HYPERPARAMETERS.items():
    #     print(f"{key}: {value}")
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
    torch.manual_seed(666) # for reproducibility
    split = [0.9, 0.1] # train, val
    train_dataset, val_dataset = torch.utils.data.random_split(dataset, split)
    # split = [0.8, 0.1, 0.1] # train, val, test
    # train_dataset, val_dataset, test_dataset = torch.utils.data.random_split(dataset, split)
    train_loader = DataLoader(train_dataset, 
                            batch_size=batch_size, 
                            shuffle=True)
    val_loader = DataLoader(val_dataset,
                            batch_size=batch_size)

    # model
    model = PCAEncoder(model_name=MODEL_NAME, verbose=VERBOSE)

    print("Training model...")
    model.train_model(train_loader, val_loader, verbose=VERBOSE)
    print("Training complete")