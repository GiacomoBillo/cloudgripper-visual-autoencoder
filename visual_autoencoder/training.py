import torch
from tqdm import tqdm
import os
import json
from torch.utils.tensorboard import SummaryWriter
import numpy as np


DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
BREAK_LOADER = 1 # break epoch after this fraction of the loader (for quick testing)


class Trainer:
    def __init__(self, model, config):
        self.model = model
        self.optimizer = torch.optim.Adam(self.model.parameters())
        self.criterion = torch.nn.MSELoss()
        self.model.to(DEVICE)

        self.dimensions_to_learn = config["model"]["dimensions_to_learn"]

    def _train_step(self, batch):
        images, labels = batch
        images = images.to(DEVICE)
        labels = labels.to(DEVICE)
        
        self.optimizer.zero_grad()
        outputs = self.model(images)
        loss = self.criterion(outputs, labels[:,self.dimensions_to_learn])
        loss.backward()
        self.optimizer.step()

        return loss
    
    def _val_step(self, batch, verbose=False):
        images, labels = batch
        images = images.to(DEVICE)
        labels = labels.to(DEVICE)
        
        with torch.no_grad():
            outputs = self.model(images)
            loss = self.criterion(outputs, labels[:,self.dimensions_to_learn]) 

        if verbose:
            print(f"\nPredictions: x={outputs[0,0]:.2f}")#, z={outputs[0,1]:.2f}")
            print(f"Ground truth: x={labels[0,0]:.2f}")#, z={labels[0,2]:.2f}")

        return loss
    
    
    def train_model(self, 
                    train_loader, 
                    val_loader=None, 
                    early_stopping_enabled=True,
                    epochs=20, 
                    verbose=True, 
                    patience=10):
        self.model.to(DEVICE)

        writer = SummaryWriter(log_dir=self.model.model_path) # for tensorboard
        train_losses = []
        val_losses = []
        if early_stopping_enabled:
            early_stopping = self.EarlyStopping(writer, patience=patience, verbose=verbose)

        for epoch in tqdm(range(epochs), 
                          desc="Training", 
                          unit="epoch",
                          total=epochs):
            
            # training
            self.model.train()
            running_loss = 0.0
            for i, batch in enumerate(train_loader):
                loss = self._train_step(batch)
                running_loss += loss.item() 
                if i>len(train_loader)//BREAK_LOADER :
                    break
            train_loss = np.sqrt(running_loss / (len(train_loader)//BREAK_LOADER)) # from MSE to RMSE

            # print
            if verbose:
                image, label = train_loader.dataset[np.random.randint(len(train_loader.dataset))]
                image = image.unsqueeze(0)  # add batch dim
                label = label.unsqueeze(0)  # add batch dim
                # print one sample prediction vs ground truth
                loss = self._val_step((image, label), verbose=True)
                print(f"\nTraining RMSE Loss: {train_loss:.4f}")
            # logging
            writer.add_scalar("Loss/train", train_loss, epoch)
            train_losses.append(train_loss)
            self.save_learning_curve(train_losses, "train")


            # validation
            if val_loader is not None:
                self.model.eval()
                running_loss = 0.0
                for i, batch in enumerate(val_loader):
                    loss = self._val_step(batch, verbose=i==len(val_loader)-1)
                    running_loss += loss.item()
                    if i>len(val_loader)//BREAK_LOADER :
                        break
                val_loss = np.sqrt(running_loss / (len(val_loader)//BREAK_LOADER)) # from MSE to RMSE
                
                # print
                if verbose:
                    print(f'Validation RMSE Loss: {val_loss:.4f}')
                # logging
                writer.add_scalar("Loss/val", val_loss, epoch)
                val_losses.append(val_loss)
                self.save_learning_curve(val_losses, "val")
                writer.add_scalars("Learning_Curves", {"train": train_loss, "val": val_loss}, epoch)

                # early stopping
                if early_stopping_enabled:
                    if early_stopping(val_loss, epoch):
                        # restore best model
                        self.model.load_model()
                        # exit training loop
                        break
                    elif early_stopping.improved:
                        # checkpoint
                        self.model.save_model()

            # if no val_loader or early stopping not enabled (no early stopping) 
            # -> always save model
            if val_loader is None or not early_stopping_enabled: 
                # checkpoint
                self.model.save_model()
        
        writer.flush()
        writer.close()


    class EarlyStopping:
        def __init__(self, writer, patience=10, verbose=False):
            self.patience = patience
            self.verbose = verbose
            self.counter = 0
            self.best_loss = float("inf")
            self.early_stop = False
            self.improved = False
            self.writer = writer

        def __call__(self, val_loss, epoch):
            if val_loss < self.best_loss:
                self.best_loss = val_loss
                self.counter = 0
                self.improved = True
            else:
                self.counter += 1
                self.improved = False
                if self.verbose:
                    print(f"Patience count: {self.counter}/{self.patience}")
                if self.counter >= self.patience:
                    self.early_stop = True
                    self.writer.add_text("Early stopping", f"Early stopping at epoch {epoch + 1}")
                    if self.verbose:
                        print(f"Early stopping at epoch {epoch + 1}")

            return self.early_stop


    def save_learning_curve(self, losses, curve_name):
        filename = os.path.join(self.model.model_path, f'{curve_name}_curve.json')
        with open(filename, "w") as file:
            json.dump(losses, file)
