import os
import torch
from torchvision import models, transforms
from torchvision.models import ResNet18_Weights, ResNet34_Weights, ResNet50_Weights, GoogLeNet_Weights
from tqdm import tqdm
from early_stopping_pytorch import EarlyStopping
from gripper_data import GripperDataset, DataLoader
import json
import numpy as np

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
    def __init__(self, 
                 fc_layers_on_top, 
                 model_name="encoder", 
                 loss_function=None, 
                 optimizer=None, 
                 finetune_all=False,
                 architecture="ResNet18",
                 partial_epoch=1
                 ):
        self.model_name = model_name + architecture + "_fc_" + "_".join(map(str,fc_layers_on_top))
        self.model_path = os.path.abspath(os.path.join(os.path.dirname(__file__),'checkpoints',f'{self.model_name}.pt'))

        # transfer learning
        if architecture == "ResNet18":
            self.architecture = models.resnet18(weights=ResNet18_Weights.DEFAULT)
        elif architecture == "ResNet34":
            self.architecture = models.resnet34(weights=ResNet34_Weights.DEFAULT)
        elif architecture == "ResNet50":
            self.architecture = models.resnet50(weights=ResNet50_Weights.DEFAULT)
        elif architecture == "GoogLeNet":
            self.architecture = models.googlenet(weights=GoogLeNet_Weights.DEFAULT)
        else:
            raise ValueError(f"Architecture {architecture} not available")
        
        # freeze all layers
        for param in self.architecture.parameters():
            param.requires_grad = finetune_all

        self.latent_dim = fc_layers_on_top[-1]
        # add fully connected layers on top
        previous_size = self.architecture.fc.in_features
        layers = []
        for i, layer_size in enumerate(fc_layers_on_top):
            layers.append(torch.nn.Linear(previous_size, layer_size))
            if i != len(fc_layers_on_top) - 1: # linear activation for the last layer
                layers.append(torch.nn.ReLU())
            previous_size = layer_size
        self.architecture.fc = torch.nn.Sequential(*layers)

        self.partial_epoch = partial_epoch
        # loss function
        if loss_function is None:
            # L2 norm for regression
            self.loss_function = torch.nn.functional.mse_loss
        if optimizer is None:
            self.optimizer = torch.optim.Adam(self.architecture.parameters(), lr=0.001) # lr=0.001

    def forward(self, x):
        return self.architecture(x)
    
    def _train_step(self, batch):
        self.architecture.train() # set train mode

        images, state_labels = batch
        images = images.to(DEVICE)
        state_labels = state_labels.to(DEVICE)

        self.optimizer.zero_grad() # reset gradients
        prediction = self.architecture(images) # predict state from image
        # compute loss 
        loss = self.loss_function(prediction, state_labels, reduction="mean")
        loss.backward() # backpropagation
        self.optimizer.step() # update weights

        loss = loss.detach().item()
        loss_per_latent_dim = self.loss_function(prediction, state_labels, reduction="none").numpy(force=True).mean(axis=0)

        return loss, loss_per_latent_dim
    

    def _validation_step(self, batch):
        self.architecture.eval() # set eval mode

        images, state_labels = batch
        images = images.to(DEVICE)
        state_labels = state_labels.to(DEVICE)

        with torch.no_grad():
            # predict state from image
            prediction = self.architecture(images)
        
        # compute loss 
        loss = self.loss_function(prediction, state_labels, reduction="mean").detach().item()
        loss_per_latent_dim = self.loss_function(prediction, state_labels, reduction="none").numpy(force=True).mean(axis=0)

        return loss, loss_per_latent_dim


    def train_model(self, train_loader, val_loader=None, epochs=100):
        self.architecture.train() # set train mode

        train_losses = []
        val_losses = []
        train_losses_per_latent_dim = []
        val_losses_per_latent_dim = []

        early_stopping = EarlyStopping(patience=10, verbose=True, path=self.model_path)

        for epoch in tqdm(range(epochs), desc="Epochs"): 
            # train step
            train_loss_sum = 0
            train_loss_per_latent_dim_sum = np.zeros(self.latent_dim)
            num_batches = 0
            for i, batch in tqdm(enumerate(train_loader), desc=f"training epoch {epoch}", total=len(train_loader)):
                if i >= len(train_loader)//(1/self.partial_epoch): # only part of the dataset in each epoch for faster training
                    break                
                train_loss, train_loss_per_latent_dim = self._train_step(batch)
                train_loss_sum += train_loss
                train_loss_per_latent_dim_sum += train_loss_per_latent_dim
                num_batches += 1
            train_loss = train_loss_sum / num_batches
            train_loss_per_latent_dim = train_loss_per_latent_dim_sum / num_batches
            train_losses.append(train_loss)
            train_losses_per_latent_dim.append(train_loss_per_latent_dim.tolist())

            self.save_learning_curve(train_losses, "train")
            self.save_learning_curve(train_losses_per_latent_dim, "train_latent_dim")

            # validation step
            if val_loader is not None:
                val_loss_sum = 0
                val_loss_per_latent_dim_sum = np.zeros(self.latent_dim)
                num_batches = 0
                for i, batch in tqdm(enumerate(val_loader), desc=f"validation epoch {epoch}", total=len(val_loader)):
                    if i >= len(train_loader): # only part of the dataset in each epoch for faster training
                        break  
                    val_loss, val_loss_per_latent_dim = self._validation_step(batch)
                    val_loss_sum += val_loss
                    val_loss_per_latent_dim_sum += val_loss_per_latent_dim
                    num_batches += 1
                val_loss = val_loss_sum / num_batches
                val_loss_per_latent_dim = val_loss_per_latent_dim_sum / num_batches
                val_losses.append(val_loss)
                val_losses_per_latent_dim.append(val_loss_per_latent_dim.tolist())
                
                self.save_learning_curve(val_losses, "val")
                self.save_learning_curve(val_losses_per_latent_dim, "val_latent_dim")

                # early stopping and checkpoint (automatic)
                early_stopping(val_loss, self.architecture)
                if early_stopping.early_stop:
                    print(f"Early stopping: min val loss = {early_stopping.val_loss_min}")
                    # load the last checkpoint with the best model
                    self.load_model()
                    break
            
            else : 
                # manual checkpoint
                torch.save(self.architecture.state_dict(), self.model_path)
            
            print(f"Epoch {epoch} -> train_loss={train_loss}" + (f", validation_loss={val_loss}" if val_loader is not None else ""))

        if val_loader is not None:
            return train_losses, val_losses
        return train_losses
    

    def load_model(self):
        model_weights = torch.load(self.model_path, map_location=DEVICE)
        self.architecture.load_state_dict(model_weights)
        # self.resnet.load_state_dict(torch.load(self.model_path, weights_only=True))


    def save_learning_curve(self, losses, curve_name):
        filename = os.path.join(os.path.dirname(__file__),'checkpoints',f'{curve_name}_curve_{self.model_name}.json')
        with open(filename, "w") as file:
            json.dump(losses, file)
    
    def eval(self):
        self.architecture.eval()


    def eval_performance(self, data_loader):
        self.eval()

        total_loss = 0
        total_loss_per_latent_dim = np.zeros(self.latent_dim)
        for (images, state_labels) in tqdm(data_loader, desc="Evaluation"):
            images = images.to(DEVICE)
            state_labels = state_labels.to(DEVICE)

            with torch.no_grad():
                predictions = self.architecture(images)

            # compute loss 
            loss = self.loss_function(predictions, state_labels, reduction="mean").detach().item()
            loss_per_latent_dim = self.loss_function(predictions, state_labels, reduction="none").numpy(force=True).mean(axis=0)

            total_loss += loss
            total_loss_per_latent_dim += loss_per_latent_dim

        # average over the dataset
        loss = total_loss / len(data_loader)
        loss_per_latent_dim = total_loss_per_latent_dim / len(data_loader)

        return loss, loss_per_latent_dim.tolist()
    

    def get_model_path(self):
        return self.model_path
        


if __name__ == "__main__":

    architecture = "GoogLeNet"
    # hyperparameters
    fc_layers_on_top = [512, 512, 512, 5] # sizes of the fully connected layers on top of the ResNet, the last is the dimension of the output
    # fc_layers_on_top = [[5], [64, 5]]
    finetune_all = False

    # train, val, test split
    split = [0.8, 0.01, 0.19] 
    num_workers = 0

    # dataset
    batch_size = 8
    # experiment = "data_collection_clean_env"
    dataset_path = os.getenv("DATASET_PATH")
    sessions = [str(session) for session in range(1,5)] # first 20 sessions
    # images_to_load = ["Top_masked_images"]
    images_to_load = ["Images"]

    partial_epoch = 1


    # transform for resnet
    """
    from pytorch ResNet18 documentation 
    (https://pytorch.org/hub/pytorch_vision_resnet/):
        All pre-trained models expect input images normalized in the same way, 
        i.e. mini-batches of 3-channel RGB images of shape (3 x H x W), 
        where H and W are expected to be at least 224. 
        The images have to be loaded in to a range of [0, 1] and then normalized 
        using mean = [0.485, 0.456, 0.406] and std = [0.229, 0.224, 0.225]
    """
    # preprocess = transforms.Compose([
    #     transforms.ToTensor(),
    #     transforms.Resize((224,224)),
    #     # transforms.Resize(256), # resizes the shorter side of the image to 256 pixels while maintaining the aspect ratio
    #     # transforms.CenterCrop(224), # crops a 224×224 region from the center of the image 
    #     # transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]), # mean and std of ImageNet dataset used for pretraining
    #     transforms.Normalize(mean=[0.440, 0.439, 0.394], std=[0.234, 0.229, 0.244]), # normalization for original Images
    # ])

    
    preprocess = transforms.Compose([
        transforms.ToTensor(),
        transforms.Resize((224,224)),
        transforms.Normalize(mean=[0.440, 0.439, 0.394], std=[0.234, 0.229, 0.244]), # normalization for original Images
    ])
    # dataset = GripperDataset(experiment=experiment, 
    #                          sessions=sessions, 
    #                          transform=preprocess, 
    #                          images_to_load=images_to_load)
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
    test_loader = DataLoader(test_dataset, batch_size=batch_size, num_workers=num_workers)

    # Encoder
    model_name = images_to_load[0] + "_encoder"
    encoder = Encoder(
        fc_layers_on_top=fc_layers_on_top,
        model_name=model_name,
        architecture=architecture,
        finetune_all=finetune_all,
        partial_epoch=partial_epoch
    )

    # load or train
    if os.path.exists(encoder.get_model_path()):
        print("Load existing model")
        encoder.load_model()
    else:
        print("Train new model")
        train_losses, val_losses = encoder.train_model(train_loader, 
                                                       val_loader,
                                                       epochs=20
                                                       )

    # save and plot losses

    # plt.plot(train_losses, label="Training loss")
    # plt.plot(val_losses, label="Validation loss")
    # plt.xlabel("Epochs")
    # plt.ylabel("MSE Loss")
    # plt.legend()
    # plt.show()

    # evaluation on test dataset
    # test_mse = encoder.eval_performance(test_loader)
    # print(f"Model performance on test dataset: {test_mse}")


    #_________________________
    # images_to_load = ["Top_masked_images"]
    # preprocess = transforms.Compose([
    #     transforms.ToTensor(),
    #     transforms.Resize((224,224)),
    #     transforms.Normalize(mean=[0.065, 0.051, 0.023], std=[0.182, 0.143, 0.089]), # normalization for Top_masked_images
    # ])
    # dataset = GripperDataset(abs_path=dataset_path, 
    #                          sessions=sessions, 
    #                          transform=preprocess, 
    #                          images_to_load=images_to_load)
    # print(f"Number of samples in the dataset: {len(dataset)}")

    # # split data (train-test)
    # torch.manual_seed(11) # Set fixed random number seed for reproducibility
    # train_dataset, val_dataset, test_dataset = torch.utils.data.random_split(dataset, split)
    # train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=num_workers) #shuffle before training
    # val_loader = DataLoader(val_dataset, batch_size=batch_size, num_workers=num_workers)
    # test_loader = DataLoader(test_dataset, batch_size=batch_size, num_workers=num_workers)

    # # Encoder
    # model_name = images_to_load[0] + "_encoder_fc_" + "_".join(map(str,fc_layers_on_top))
    # encoder = Encoder(
    #     fc_layers_on_top=fc_layers_on_top,
    #     model_name=model_name
    # )

    # # load or train
    # if os.path.exists(encoder.get_model_path()):
    #     print("Load existing model")
    #     encoder.load_model()
    # else:
    #     print("Train new model")
    #     train_losses, val_losses = encoder.train_model(train_loader, 
    #                                                    val_loader,
    #                                                    epochs=20
    #                                                    )