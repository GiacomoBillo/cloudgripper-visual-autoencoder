import torch
from torchvision import transforms
from tqdm import tqdm
import os
from gripper_data import GripperDataset, DataLoader

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

"""
5D configuration of the robot -> 1 RGB pixel prediction
"""
class PixelGenerator(torch.nn.Module):
    def __init__(self, 
                 coordinates: tuple, 
                 layers: list,
                 input_dim = 5,
                 activation_function = torch.nn.ReLU(),
                 loss_function = None,
                 optimizer = None,
                 *args, 
                 **kwargs):
        super().__init__(*args, **kwargs)

        self.coordinates = coordinates
        self.input_dim = input_dim
        self.output_size = 3 # RGB
        self.activation_function = activation_function

        architecture = []
        
        in_dim = input_dim
        # add custom fully connected layers
        for size in layers:
            architecture.append(torch.nn.Linear(in_dim,size))
            architecture.append(self.activation_function) # ReLU by default
            in_dim = size
        # add output layer
        architecture.append(torch.nn.Linear(layers[-1],self.output_size))
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
    
    def train_model(self, train_loader, epochs=1, verbose=False):
        self.train()

        train_losses = []

        for epoch in range(epochs):
            for i, batch in tqdm(enumerate(train_loader), total=len(train_loader)):
                img, states = batch
                # select target pixel -> (all batch, RGB, i, j)
                target = img[:,:,*self.coordinates].to(DEVICE)
                states = states.to(DEVICE)

                prediction = self.forward(states)
                loss = self.loss_function(prediction, target)
                loss.backward()
                self.optimizer.step()

                if i%100==0:
                    train_losses.append(loss.detach().numpy())
                    if verbose:
                        print(f"Batch iteration {i} -> train loss = {loss}")

        return train_losses        

    def evaluate(self, test_loader):
        self.eval()

        tot_loss = 0
        for i, batch in enumerate(test_loader):
            img, states = batch
            # select target pixel
            target = img[:,:,*self.coordinates].to(DEVICE)
            states = states.to(DEVICE)

            with torch.no_grad():
                prediction = self.forward(states)
                loss = self.loss_function(prediction, target)
                
            tot_loss += loss

        return tot_loss/len(test_loader)

    

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
    train_losses = model.train_model(train_loader)
    print(train_losses)