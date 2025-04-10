import torch
from torchvision import transforms
from tqdm import tqdm
import os
from gripper_data import GripperDataset, DataLoader
import json


DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

"""
5D configuration of the robot -> 1 RGB pixel prediction
"""
class PixelGenerator(torch.nn.Module):
    def __init__(self, 
                 coordinates: tuple, 
                 hidden_layers: list,
                 input_dim: int = 5,
                 activation_function = torch.nn.ReLU(),
                 loss_function = None,
                 optimizer = None,
                 additional_name = "",
                 *args, 
                 **kwargs):
        super().__init__(*args, **kwargs)

        self.model_name = "pixel_decoder_" + "_".join(map(str,coordinates)) + "_layers_" + "_".join(map(str,hidden_layers)) + additional_name
        self.path = os.path.abspath(os.path.join(os.path.dirname(__file__),'decoder_checkpoints',f'{self.model_name}'))
        os.makedirs(self.path, exist_ok=True)   

        self.coordinates = coordinates
        self.input_dim = input_dim
        self.output_size = 3 # RGB
        self.activation_function = activation_function

        architecture = []
        
        in_dim = input_dim
        # add custom fully connected layers
        for size in hidden_layers:
            architecture.append(torch.nn.Linear(in_dim,size))
            architecture.append(self.activation_function) # ReLU by default
            in_dim = size
        # add output layer
        architecture.append(torch.nn.Linear(hidden_layers[-1],self.output_size))
        architecture.append(torch.nn.Sigmoid()) # constrain output in [0,1]
        self.architecture = torch.nn.Sequential(*architecture)

        if loss_function is None:
            # L2 norm for regression
            self.loss_function = torch.nn.MSELoss()
        if optimizer is None:
            self.optimizer = torch.optim.Adam(self.architecture.parameters(), lr=0.001)
        
    def forward(self, x):
        prediction = self.architecture(x)
        return prediction
    
    def train_model(self, train_loader, val_loader=None, epochs=20, verbose=False):
        train_losses = []
        val_losses = []

        for epoch in tqdm(range(epochs), desc="Epochs"):
            self.train()
            running_loss = 0
            for i, batch in tqdm(enumerate(train_loader), total=len(train_loader), desc=f"Training epoch {epoch}"):
                img, states = batch
                # select target pixel -> (all batch, RGB, i, j)
                target = img[:,:,*self.coordinates].to(DEVICE)
                states = states.to(DEVICE)

                self.optimizer.zero_grad() # reset gradients
                prediction = self.forward(states)
                loss = self.loss_function(prediction, target)
                loss.backward()
                self.optimizer.step()

                running_loss += loss.detach().item()
            # store train loss
            train_loss = running_loss/len(train_loader)
            if verbose:
                print("training loss:", train_loss)
            train_losses.append(train_loss)
            self.save_learning_curve(train_losses, curve_name="training_losses")
            
            if val_loader is not None:
                self.eval()
                val_loss = self.evaluate(val_loader, description=f"Validation epoch {epoch}")
                val_losses.append(val_loss)
                # store val loss
                self.save_learning_curve(val_losses, curve_name="validation_losses")
                if verbose:
                    print("validation loss", val_loss)

            # checkpoint -> store model parameters for each epoch
            torch.save(self.architecture.state_dict(), 
                       os.path.join(self.path, f"model_epoch_{epoch}.pt"))

        return train_losses        

    def evaluate(self, test_loader, description=""):
        self.eval()

        tot_loss = 0
        for i, batch in tqdm(enumerate(test_loader), total=len(test_loader), desc=description):
            img, states = batch
            # select target pixel
            target = img[:,:,*self.coordinates].to(DEVICE)
            states = states.to(DEVICE)

            with torch.no_grad():
                prediction = self.forward(states)
                loss = self.loss_function(prediction, target)
                
            tot_loss += loss.item()

        return tot_loss/len(test_loader)
    
    def save_learning_curve(self, losses, curve_name):
        filename = os.path.join(self.path, f'{curve_name}.json')
        with open(filename, "w") as file:
            json.dump(losses, file)

    def get_name(self):
        return self.model_name
    
    def load_model(self, epoch=20):
        checkpoint_path = os.path.join(self.path,f"model_epoch_{epoch}.pt")
        self.architecture.load_state_dict(torch.load(checkpoint_path, 
                                                     map_location=DEVICE))

    

if __name__ == "__main__" :
    # train, val, test split
    split = [0.8, 0.1, 0.1] 
    num_workers = 0

    # dataset
    batch_size = 10
    dataset_path = os.getenv("DATASET_PATH")
    sessions = [str(session) for session in range(1,21)] # first 20 sessions
    images_to_load = ["Images"]

    preprocess = transforms.Compose([
        transforms.ToTensor(),
    ])
    dataset = GripperDataset(abs_path=dataset_path, 
                             sessions=sessions, 
                             transform=preprocess, 
                             images_to_load=images_to_load)
    print(f"Number of samples in the dataset: {len(dataset)}")

    # split data (train-test)
    torch.manual_seed(11) # Set fixed random number seed for reproducibility
    train_dataset, val_dataset, test_dataset = torch.utils.data.random_split(dataset, split)
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=num_workers) #shuffle before training
    val_loader = DataLoader(val_dataset, batch_size=batch_size, num_workers=num_workers)

    pixel_coordinates = (100,100)
    layers = [32, 32]
    model = PixelGenerator(pixel_coordinates, layers)
    train_losses, val_losses = model.train_model(train_loader, val_loader)
    print(train_losses)
    print(val_losses)