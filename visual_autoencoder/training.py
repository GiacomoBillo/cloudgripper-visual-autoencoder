import torch
from tqdm import tqdm
import os
import json
from torch.utils.tensorboard import SummaryWriter
from torch.utils.data import DataLoader
import numpy as np
from architecture import AcceleratedArchitecture
import time
from torchmetrics.image.lpip import LearnedPerceptualImagePatchSimilarity as LPIPSLoss


BREAK_LOADER = 1 # break epoch after this fraction of the loader (for quick testing)


class Trainer:
    def __init__(self, model: AcceleratedArchitecture, config):
        self.model = model
        self.accelerator = model.accelerator # unwrap accelerator 
        self.logger = self.model.logger # unwrap logger

        self.dimensions_to_learn = config["model"]["dimensions_to_learn"]

        # learning rate
        self.lr = config["training"]["learning_rate"]
        if self.lr is None:
            self.lr = 0.001

        # optimizer
        optimizer_type = config["training"]["optimizer"]
        if optimizer_type is None or optimizer_type == "Adam":
            self.optimizer = torch.optim.Adam(self.model.parameters(), lr=self.lr)
        elif optimizer_type == "SGD":
            self.optimizer = torch.optim.SGD(self.model.parameters(), lr=self.lr, momentum=0.9)
        else:
            raise ValueError(f"Unknown optimizer type: {optimizer_type}")

        # loss function
        self.loss_type = config["training"]["loss_function"]
        if self.loss_type is None or self.loss_type == "MSE":
            self.loss_fn = torch.nn.MSELoss() 
            self.loss_type = "RMSE" # train with MSE, report RMSE
        elif "decoder" in self.model.model_type and self.loss_type == "LPIPS":
            # vgg more accurate better for backprop, alex faster
            net_type = "vgg"
            # normalize inputs from [0,1] (ours) to [-1,1] (for LPIPS)
            self.loss_fn = LPIPSLoss(net_type=net_type, normalize=True)
            self.logger.print(f"\nLPIPS loss with {net_type}")
        else:
            raise ValueError(f"Unknown loss function type: {self.loss_type}, for model type: {self.model.model_type}")


    def _inference_step(self, batch, verbose=False):
        images, labels = batch
        # images = images.to(DEVICE) done implicitly by accelerator
        # labels = labels.to(DEVICE)

        # decide input and target based on model type
        if "encoder" in self.model.model_type:
            outputs = self.model(images)
            loss = self.loss_fn(outputs, labels[:,self.dimensions_to_learn])

            if verbose:
                print(f"\nPredictions: x={outputs[0,0]:.2f}")#, z={outputs[0,1]:.2f}")
                print(f"Ground truth: x={labels[0,0]:.2f}")#, z={labels[0,2]:.2f}")
        
        elif "decoder" in self.model.model_type:
            outputs = self.model(labels[:,self.dimensions_to_learn])
            loss = self.loss_fn(outputs, images)

            # TODO: if verbose plot reconstructed image vs input image
        else:
            raise ValueError(f"Unknown model type: {self.model.model_type}")
        
        return outputs, loss


    def _train_step(self, batch):        
        self.optimizer.zero_grad()
        outputs, loss = self._inference_step(batch)
        self.accelerator.backward(loss)  # instead of loss.backward()
        self.optimizer.step()
        return loss
    
    
    def _val_step(self, batch, verbose=False):
        with torch.no_grad():
            outputs, loss = self._inference_step(batch, verbose)
        return loss
    
    
    def train_model(self, 
                    train_loader: DataLoader, 
                    val_loader: DataLoader=None, 
                    early_stopping_enabled=True,
                    epochs=20, 
                    verbose=True, 
                    patience=10):
        
        # prepare for accelerator -> implicit to device (cpu, gpu or multi-gpu)
        self.model, self.optimizer, self.loss_fn, train_loader, val_loader = self.accelerator.prepare(self.model, self.optimizer, self.loss_fn, train_loader, val_loader)

        writer = SummaryWriter(log_dir=self.model.model_path) # for tensorboard
        train_losses = []
        val_losses = []
        if early_stopping_enabled:
            early_stopping = self.EarlyStopping(writer, self.model.logger, patience=patience)

        self.logger.print("\n\nStarting training...", )
        start_time = time.time()

        for epoch in tqdm(range(epochs), 
                          desc="Training", 
                          unit="epoch",
                          total=epochs):
            self.logger.print(f"\nEpoch {epoch+1}/{epochs}")
            
            # training
            self.model.train()
            running_loss = 0.0
            for i, batch in enumerate(train_loader):
                loss = self._train_step(batch)
                running_loss += loss.item() 
                if i>len(train_loader)//BREAK_LOADER :
                    break
            train_loss = running_loss / (len(train_loader)//BREAK_LOADER)
            if self.loss_type == "RMSE":
                train_loss = np.sqrt(train_loss) # from MSE to RMSE

            # print
            if verbose:
                image, label = train_loader.dataset[np.random.randint(len(train_loader.dataset))]
                image = image.unsqueeze(0)  # add batch dim
                label = label.unsqueeze(0)  # add batch dim
                # print one sample prediction vs ground truth
                loss = self._val_step((image, label), verbose=True)
            self.logger.print(f"Training {self.loss_type} loss: {train_loss:.4f}")
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
                val_loss = running_loss / (len(val_loader)//BREAK_LOADER) 
                if self.loss_type == "RMSE":
                    val_loss = np.sqrt(val_loss) # from MSE to RMSE
                
                # print
                self.logger.print(f'Validation {self.loss_type} loss: {val_loss:.4f}')
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

        total_time = time.time() - start_time
        hrs, secs = divmod(total_time, 3600)
        mins, secs = divmod(secs, 60)
        self.logger.print(f"Training finished"\
                          f"\n\t- time = {int(hrs)}h {int(mins)}m {int(secs)}s"\
                          f"\n\t- best validation {self.loss_type} loss = {early_stopping.best_loss:.4f}" if early_stopping is not None else "")
        self.logger.flush()
        writer.flush()
        writer.close()


    class EarlyStopping:
        def __init__(self, writer, logger, patience=10):
            self.patience = patience
            self.logger = logger
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
                self.logger.print(f"Patience count: {self.counter}/{self.patience}")
                if self.counter >= self.patience:
                    self.early_stop = True
                    self.writer.add_text("Early stopping", f"Early stopping at epoch {epoch + 1}")
                    self.logger.print(f"Early stopping at epoch {epoch + 1}")

            return self.early_stop


    def save_learning_curve(self, losses, curve_name):
        filename = os.path.join(self.model.model_path, f'{curve_name}_curve.json')
        with open(filename, "w") as file:
            json.dump(losses, file)
