import os
import torch
from torchvision import models, transforms
from torchvision.models import ResNet18_Weights
from tqdm import tqdm
from early_stopping_pytorch import EarlyStopping
from gripper_data import GripperDataset, DataLoader
import cv2
import json

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

"""
Encoder

Input:
- top image
- (bottom image)
- mask

Output (= Latent space of the AE):
- 5D state of the robot (x,y,z,r,g)

Training: 
- transfer learning from a pre-trained model
- add layer(s) on top
- supervised fine-tuning

Model selection comparing: 
- different architectures 
- different inputs (top, top+bottom, partial)
"""


"""
Transfer learning
the input size doesn't matter since the first layer is a convolutional layer
after the first layer, the pooling layer fix the size
"""


# Encoder from image (1 channel) to robot state
class Encoder():
    def __init__(self, fc_layers_on_top, model_name="encoder"):
        self.model_name = model_name
        self.model_path = os.path.join('checkpoints',f'{self.model_name}.pt')

        # transfer learning
        self.resnet = models.resnet18(weights=ResNet18_Weights.DEFAULT)
        # freeze all layers
        for param in self.resnet.parameters():
            param.requires_grad = False

        # add fully connected layers on top
        previous_size = self.resnet.fc.in_features
        layers = []
        for i, layer_size in enumerate(fc_layers_on_top):
            layers.append(torch.nn.Linear(previous_size, layer_size))
            if i != len(fc_layers_on_top) - 1: # linear activation for the last layer
                layers.append(torch.nn.ReLU())
            previous_size = layer_size
        self.resnet.fc = torch.nn.Sequential(*layers)

    def forward(self, x):
        return self.resnet(x)
    
    def _train_step(self, batch, loss_function, optimizer: torch.optim.Optimizer):
        self.resnet.train() # set train mode

        images, state_labels = batch
        images = images.to(DEVICE)
        state_labels = state_labels.to(DEVICE)

        optimizer.zero_grad() # reset gradients
        prediction = self.resnet(images) # predict state from image
        loss = loss_function(prediction, state_labels) # compute loss
        loss.backward() # backpropagation
        optimizer.step() # update weights

        return loss.item() # convert tensor to scalar
    

    def _validation_step(self, batch, loss_function):
        self.resnet.eval() # set eval mode

        images, state_labels = batch
        images = images.to(DEVICE)
        state_labels = state_labels.to(DEVICE)

        with torch.no_grad():
            # predict state from image
            prediction = self.resnet(images)
            # compute loss 
            loss = loss_function(prediction, state_labels)

        return loss.item() # convert tensor to scalar


    def train_model(self, train_loader, val_loader=None, epochs=100, loss_function=None, optimizer=None):
        self.resnet.train() # set train mode

        if loss_function is None:
            loss_function = torch.nn.MSELoss() # L2 norm for regression
        if optimizer is None:
            optimizer = torch.optim.Adam(self.resnet.parameters(), lr=0.001)

        train_losses = []
        val_losses = []
        early_stopping = EarlyStopping(patience=10, verbose=True, path=self.model_path)

        for epoch in tqdm(range(epochs), desc="Epochs"): 
            # train step
            train_loss = 0
            for i, batch in tqdm(enumerate(train_loader), desc=f"training epoch {epoch}", total=len(train_loader), leave=False):
                train_loss += self._train_step(batch, loss_function, optimizer)
            train_loss /= len(train_loader)
            train_losses.append(train_loss)

            # validation step
            if val_loader is not None:
                val_loss = 0
                for i, batch in tqdm(enumerate(val_loader), desc=f"validation epoch {epoch}", total=len(val_loader), leave=False):
                    val_loss += self._validation_step(batch, loss_function)
                val_loss /= len(val_loader)
                val_losses.append(val_loss)

            # checkpoint
            # os.makedirs("checkpoints", exist_ok=True)
            # filename = f"checkpoints/{self.model_name}_epoch_{epoch}.pth"
            # torch.save({
            #             "epoch": epoch,
            #             "model_state_dict": self.resnet.state_dict(), 
            #             "optimizer_state_dict": optimizer.state_dict(),
            #             "loss": loss
            #             },
            #             filename)
            # print(f"Checkpoint epoch {epoch} saved at {filename}")
            print(f"Epoch {epoch} -> train_loss={train_loss}" + (f", validation_loss={val_loss}" if val_loader is not None else ""))

            # early stopping
            early_stopping(val_loss, self.resnet)
            if early_stopping.early_stop:
                print(f"Early stopping: min val loss = {early_stopping.val_loss_min}")
                break
        
        # load the last checkpoint with the best model
        self.load_model()

        self.save_learning_curve(train_losses, "training")
        if val_loader is not None:
            self.save_learning_curve(val_losses, "validation")
            return train_losses, val_losses
        return train_losses
    

    def load_model(self):
        self.resnet.load_state_dict(torch.load(self.model_path, weights_only=True))


    def save_learning_curve(self, losses, curve_name):
        filename = os.path.join('checkpoints',f'{curve_name}_curve_{self.model_name}.json')
        with open(filename, "w") as file:
            json.dump(losses, file)
    
    def eval(self):
        self.resnet.eval()


    def eval_performance(self, data_loader, loss_function=None):
        self.eval()

        if loss_function is None:
            loss_function = torch.nn.MSELoss()

        total_loss = 0
        for (bottom_images, top_images, state_labels) in data_loader:
            top_images = top_images.to(DEVICE)
            state_labels = state_labels.to(DEVICE)

            input_images = self._process_input(top_images)

            with torch.no_grad():
                predictions = self.resnet(input_images)
            total_loss += loss_function(predictions, state_labels).item()

        return total_loss / len(data_loader)
    

    def get_model_path(self):
        return self.model_path
        


if __name__ == "__main__":

    # transform needed for resnet
    preprocess = transforms.Compose([
        # transforms.Resize(256),
        # transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])

    # dataset
    batch_size = 10
    experiment = "data_collection_clean_env"
    sessions = [str(session) for session in range(1,21)] # first 20 sessions
    images_to_load = ["Top_masked_images"]
    # images_to_load = ["Images"]
    dataset = GripperDataset(experiment=experiment, sessions=sessions, transform=preprocess)
    # dataset = GripperDataset(abs_path=path, sessions=sessions, transform=preprocess)
    
    # split data (train-test)
    train_dataset, val_dataset, test_dataset = torch.utils.data.random_split(dataset, [0.8, 0.1, 0.1])
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True) #shuffle before training
    val_loader = DataLoader(val_dataset, batch_size=batch_size)
    test_loader = DataLoader(test_dataset, batch_size=batch_size)


    # hyperparameters
    fc_layers_on_top = [5] # sizes of the fully connected layers on top of the ResNet, the last is the dimension of the output


    # Encoder
    model_name = "encoder_fc_" + "_".join(map(str,fc_layers_on_top))
    encoder = Encoder(
        fc_layers_on_top=fc_layers_on_top,
        model_name=model_name
    )

    # load or train
    if os.path.exists(encoder.get_model_path()):
        print("Load existing model")
        encoder.load_model()
    else:
        print("Train new model")
        train_losses, val_losses = encoder.train_model(train_loader, val_loader)

    # save and plot losses

    # plt.plot(train_losses, label="Training loss")
    # plt.plot(val_losses, label="Validation loss")
    # plt.xlabel("Epochs")
    # plt.ylabel("MSE Loss")
    # plt.legend()
    # plt.show()

    # evaluation on test dataset
    test_mse = encoder.eval_performance(test_loader)
    print(f"Model performance on test dataset: {test_mse}")

