import os
import torch
from torchvision import models
from torchvision.models import ResNet18_Weights
from tqdm import tqdm
from early_stopping_pytorch import EarlyStopping
from visual_autoencoder.data_loader import MaskEngine

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


# Encoder from image to robot state
class Encoder():
    def __init__(self, fc_layers_on_top, model_name="encoder", encoder_from_mask=False, mask_engine_top=None):
        self.model_name = model_name
        self.encoder_from_mask = encoder_from_mask
        if encoder_from_mask:
            assert isinstance(mask_engine_top, MaskEngine)
            self.mask_engine_top = mask_engine_top

        # transfer learning
        self.resnet = models.resnet18(weights=ResNet18_Weights.DEFAULT)

        # add fully connected layers on top
        self.resnet.fc = torch.nn.Sequential()
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
    
    def _process_input(self, images):
        if self.encoder_from_mask:
            # online computation of the mask
            masks = self.mask_engine_top.create_mask(images)
            masked_images = self.mask_engine_top.apply_mask(images, masks)
            return masked_images
        else:
            return images
    
    def _train_step(self, batch, loss_function, optimizer: torch.optim.Optimizer):
        self.resnet.train() # set train mode

        bottom_images, top_images, state_labels = batch
        bottom_images = bottom_images.to(DEVICE)
        top_images = top_images.to(DEVICE)
        state_labels = state_labels.to(DEVICE)

        input_images = self._process_input(top_images)

        optimizer.zero_grad() # reset gradients
        prediction = self.resnet(input_images)
        loss = loss_function(prediction, state_labels)
        loss.backward() # backpropagation
        optimizer.step() # update weights

        return loss.item() # convert tensor to scalar
    

    def _validation_step(self, batch, loss_function):
        self.resnet.eval() # set eval mode

        bottom_images, top_images, state_labels = batch
        bottom_images = bottom_images.to(DEVICE)
        top_images = top_images.to(DEVICE)
        state_labels = state_labels.to(DEVICE)

        input_images = self._process_input(top_images)

        with torch.no_grad():
            prediction = self.resnet(input_images)
            loss = loss_function(prediction, state_labels)

        return loss.item() # convert tensor to scalar


    def train_model(self, train_loader, val_loader=None, epochs=100, from_masked_robot=False, loss_function=None, optimizer=None):
        self.resnet.train() # set train mode

        if loss_function is None:
            loss_function = torch.nn.MSELoss() # L2 norm for regression
        if optimizer is None:
            optimizer = torch.optim.Adam(self.resnet.parameters(), lr=0.001)

        train_losses = []
        val_losses = []
        early_stopping = EarlyStopping(patience=5, verbose=True, path='checkpoint.pt')

        for epoch in tqdm(range(epochs), desc="Epochs"): 
            # train step
            train_loss = 0
            for i, batch in tqdm(enumerate(train_loader)):
                train_loss += self._train_step(batch, from_masked_robot, loss_function, optimizer)   
            train_losses.append(train_loss)

            # validation step
            if val_loader is not None:
                val_loss = 0
                for i, batch in tqdm(enumerate(val_loader)):
                    val_loss += self._validation_step(batch, loss_function)
                val_losses.append(val_loss)
                loss = train_loss, val_loss
            else:
                loss = train_loss   

            # checkpoint
            os.makedirs("checkpoints", exist_ok=True)
            filename = f"checkpoints/{self.model_name}_epoch_{epoch}.pth"
            torch.save({
                        "epoch": epoch,
                        "model_state_dict": self.resnet.state_dict(), 
                        "optimizer_state_dict": optimizer.state_dict(),
                        "loss": loss
                        },
                        filename)
            print(f"Checkpoint epoch {epoch} saved at {filename}")
            print(f"Epoch {epoch}, loss: {loss}")

            # early stopping
            early_stopping(val_loss, self.resnet)
            if early_stopping.early_stop:
                print(f"Early stopping: min val loss = {early_stopping.val_loss_min}")
                break
        
        # load the last checkpoint with the best model
        self.resnet.load_state_dict(torch.load('checkpoint.pt', weights_only=True))

        if val_loader is not None:
            return train_losses, val_losses
        return train_losses
    
    
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


    @staticmethod
    def train_or_load():
        pass



if __name__ == "__main__":

    # transform needed for resnet
    from torchvision import transforms
    preprocess = transforms.Compose([
        # transforms.Resize(256),
        # transforms.CenterCrop(224),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])

