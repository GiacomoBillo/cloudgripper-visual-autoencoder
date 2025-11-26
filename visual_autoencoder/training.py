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
from torchmetrics.image import PeakSignalNoiseRatio as PSNRLoss, StructuralSimilarityIndexMeasure as SSIMLoss
from accelerate.utils import broadcast

from gripper_data_ref import GripperDatasetReference
from utils import ENCODERS, DECODERS, ARCHITECTURES_WITH_REFERENCE, DIMENSIONS


class Trainer:
    def __init__(self, model: AcceleratedArchitecture, config):
        self.model = model
        self.model_path = model.model_path
        self.model_type = model.model_type
        self.accelerator = model.accelerator # unwrap accelerator 
        self.logger = model.logger # unwrap logger
        
        if type(model).__name__ in ARCHITECTURES_WITH_REFERENCE:
            self.reference = True
            self.delta = model.delta if hasattr(model, "delta") else False
        else:
            self.reference = None

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

        # loss function for training
        self.loss_type = config["training"]["loss_function"]
        if self.loss_type is None or self.loss_type == "MSE":
            self.loss_fn = torch.nn.MSELoss() 
            self.loss_type = "RMSE" # train with MSE, report RMSE
        elif "decoder" in self.model_type and self.loss_type == "LPIPS":
            # vgg more accurate better for backprop, alex faster
            net_type = "vgg"
            # normalize inputs from [0,1] (ours) to [-1,1] (for LPIPS)
            self.loss_fn = LPIPSLoss(net_type=net_type, normalize=True)
            self.logger.print(f"\nLPIPS loss with {net_type}")
        else:
            raise ValueError(f"Unknown loss function type: {self.loss_type}, for model type: {self.model_type}")

        # loss functions for evaluation
        self.eval_loss_functions = self.set_eval_metrics()
        

    def _inference_step(self, batch, verbose=False):
        # decoder with reference
        if self.reference is not None:
            images, labels, reference_images, reference_labels  = batch
            if self.delta:
                labels = labels - reference_labels

            outputs = self.model(reference_images, labels[:,self.dimensions_to_learn])
            loss = self.loss_fn(outputs, images)

        else:
            images, labels = batch
            # images = images.to(DEVICE) done implicitly by accelerator
            # labels = labels.to(DEVICE)

            # decide input and target based on model type
            if "encoder" in self.model_type:
                outputs = self.model(images)
                loss = self.loss_fn(outputs, labels[:,self.dimensions_to_learn])

                if verbose:
                    print(f"\nPredictions: x={outputs[0,0]:.2f}")#, z={outputs[0,1]:.2f}")
                    print(f"Ground truth: x={labels[0,0]:.2f}")#, z={labels[0,2]:.2f}")
            
            elif "decoder" in self.model_type:
                outputs = self.model(labels[:,self.dimensions_to_learn])
                loss = self.loss_fn(outputs, images)

                # TODO: if verbose plot reconstructed image vs input image
            else:
                raise ValueError(f"Unknown model type: {self.model_type}")
        
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
    
    def _on_main_save_model(self):
        self.accelerator.wait_for_everyone()  #  sync all processes
        if self.accelerator.is_main_process:
            self.accelerator.unwrap_model(self.model).save_model() # unwrap to original model and save
        self.accelerator.wait_for_everyone() 

    def _on_main_load_model(self):
        self.accelerator.wait_for_everyone()  #  sync all processes
        if self.accelerator.is_main_process:
            self.model = self.accelerator.unwrap_model(self.model).load_model() # load model
            self.model = self.accelerator.prepare(self.model)  # re-prepare model with accelerator
        self.accelerator.wait_for_everyone() 

    def _on_main_print(self, msg):
        if self.accelerator.is_main_process:
            self.logger.print(msg)

    def gather_loss(self, running_loss: torch.Tensor, loader_length):
        # gather loss from all gpus
        total_batches = loader_length * self.accelerator.num_processes
        loss_sum = self.accelerator.gather(running_loss).sum().item()
        loss = loss_sum / total_batches
        if self.loss_type == "RMSE":
            loss = np.sqrt(loss) # from MSE to RMSE
        return loss
    
    def _broadcast_bool(self, value):
        # Convert bools to tensor
        tensor = torch.tensor(int(value), device=self.accelerator.device)
        # Broadcast from main
        tensor = broadcast(tensor)
        # Convert back to bools
        bool_value = bool(tensor.item())
        return bool_value


    def train_model(self, 
                    train_loader: DataLoader, 
                    val_loader: DataLoader=None, 
                    early_stopping_enabled=True,
                    epochs=20, 
                    patience=10):
        
        # save indices of reference images
        if self.reference:
            self.logger.print(f"Indices of reference images: {train_loader.dataset.dataset.dataset.ref_indices.tolist()}")
            # save to file
            ref_indices_file = os.path.join(self.model_path, "reference_indices.json")
            with open(ref_indices_file, "w") as f:
                json.dump(train_loader.dataset.dataset.dataset.ref_indices.tolist(), f)
            self.logger.print(f"Reference indices saved to file {ref_indices_file}\n")

        if self.accelerator.num_processes > 1:
            self.logger.print(f"Using {self.accelerator.num_processes} processes for training.")
            self._train_distributed_model(train_loader, val_loader, early_stopping_enabled, epochs, patience)

        else:
            self.logger.print(f"Using single process for training, device: {self.accelerator.device}")
            self._train_single_model(train_loader, val_loader, early_stopping_enabled, epochs, patience)


    def _train_single_model(self, train_loader, val_loader, early_stopping_enabled, epochs, patience):
        # prepare for accelerator -> implicit to device (cpu, gpu or multi-gpu)
        self.model, self.optimizer, self.loss_fn, train_loader = self.accelerator.prepare(self.model, self.optimizer, self.loss_fn, train_loader)
        if val_loader:
            val_loader = self.accelerator.prepare(val_loader)
            
        writer = SummaryWriter(log_dir=self.model.model_path) # for tensorboard
        train_losses = []
        val_losses = []
        if early_stopping_enabled:
            early_stopping = self.EarlyStopping(writer, self.logger, patience=patience)

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
            for i, batch in tqdm(enumerate(train_loader), desc="Training", total=len(train_loader)):
                loss = self._train_step(batch)
                running_loss += loss.item() 

            train_loss = running_loss / len(train_loader)
            if self.loss_type == "RMSE":
                train_loss = np.sqrt(train_loss) # from MSE to RMSE


            self.logger.print(f"Training {self.loss_type} loss: {train_loss:.4f}")
            # logging
            writer.add_scalar("Loss/train", train_loss, epoch)
            train_losses.append(train_loss)
            self.save_learning_curve(train_losses, "train")


            # validation
            if val_loader is not None:
                self.model.eval()
                running_loss = 0.0
                for i, batch in tqdm(enumerate(val_loader), desc="Validation", total=len(val_loader)):
                    loss = self._val_step(batch, verbose=i==len(val_loader)-1)
                    running_loss += loss.item()

                val_loss = running_loss / len(val_loader)
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
                          f"\n\t- best validation {self.loss_type} loss = {early_stopping.best_loss:.4f}" if val_loader and early_stopping_enabled else "")
        self.logger.flush()
        writer.flush()
        writer.close()


    def _train_distributed_model(self, train_loader, val_loader, early_stopping_enabled, epochs, patience):

        writer = SummaryWriter(log_dir=self.model_path) # for tensorboard
        train_losses = []
        val_losses = []
        if early_stopping_enabled:
            early_stopping = self.EarlyStopping(writer, self.logger, patience=patience)

        self._on_main_print("\n\nStarting training...", )
        start_time = time.time()

        # prepare for accelerator -> implicit to device (cpu, gpu or multi-gpu)
        self.model, self.optimizer, self.loss_fn, train_loader = self.accelerator.prepare(self.model, self.optimizer, self.loss_fn, train_loader)
        if val_loader:
            val_loader = self.accelerator.prepare(val_loader)

        for epoch in tqdm(range(epochs), 
                          desc="Training", 
                          unit="epoch",
                          total=epochs):
            self._on_main_print(f"\nEpoch {epoch+1}/{epochs}")

            # training
            self.model.train()
            running_loss = torch.tensor(0.0, device=self.accelerator.device)
            for i, batch in enumerate(train_loader):
                loss = self._train_step(batch)
                running_loss += loss

            # gather loss from all gpus
            train_loss = self.gather_loss(running_loss, len(train_loader))
            
            self._on_main_print(f"Training {self.loss_type} loss: {train_loss:.4f}")
            # logging
            if self.accelerator.is_main_process:
                writer.add_scalar("Loss/train", train_loss, epoch)
                train_losses.append(train_loss)
                self.save_learning_curve(train_losses, "train")


            # validation
            if val_loader is not None:
                self.model.eval()
                running_loss = torch.tensor(0.0, device=self.accelerator.device)
                for i, batch in enumerate(val_loader):
                    loss = self._val_step(batch, verbose=i==len(val_loader)-1)
                    running_loss += loss

                # gather loss from all gpus
                val_loss = self.gather_loss(running_loss, len(val_loader))
                
                # print
                self._on_main_print(f'Validation {self.loss_type} loss: {val_loss:.4f}')
                # logging
                if self.accelerator.is_main_process:
                    writer.add_scalar("Loss/val", val_loss, epoch)
                    val_losses.append(val_loss)
                    self.save_learning_curve(val_losses, "val")
                    writer.add_scalars("Learning_Curves", {"train": train_loss, "val": val_loss}, epoch)

                # early stopping
                if early_stopping_enabled:
                    # check early stopping on main process
                    if self.accelerator.is_main_process:
                        stop_training = early_stopping(val_loss, epoch)
                        improved = early_stopping.improved
                    else:
                        stop_training = False
                        improved = False

                    stop_training = self._broadcast_bool(stop_training)
                    improved = self._broadcast_bool(improved)
                    self.accelerator.wait_for_everyone()

                    if stop_training:
                        # restore best model
                        self._on_main_load_model()
                        # exit training loop
                        break
                    elif improved:
                        # checkpoint
                        self._on_main_save_model()

            # if no val_loader or early stopping not enabled (no early stopping) 
            # -> always save model
            if val_loader is None or not early_stopping_enabled: 
                # checkpoint
                self._on_main_save_model()

        total_time = time.time() - start_time
        hrs, secs = divmod(total_time, 3600)
        mins, secs = divmod(secs, 60)
        self._on_main_print(f"Training finished"\
                          f"\n\t- time = {int(hrs)}h {int(mins)}m {int(secs)}s"\
                          f"\n\t- best validation {self.loss_type} loss = {early_stopping.best_loss:.4f}" if val_loader and early_stopping_enabled else "")
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
        filename = os.path.join(self.model_path, f'{curve_name}_curve.json')
        with open(filename, "w") as file:
            json.dump(losses, file)


    def set_eval_metrics(self):
        if self.model.__class__.__name__ in ENCODERS:
            def MSE_per_dimension(outputs, targets):
                mse = torch.nn.MSELoss(reduction='none')(outputs, targets)
                mse_per_dim = torch.mean(mse, dim=0)  # mean over batch
                return mse_per_dim
            
            eval_metrics = {
                "MSE": torch.nn.MSELoss(),
                "MSE per dimension": MSE_per_dimension,
            }

        elif self.model.__class__.__name__ in DECODERS:
            eval_metrics = {
                "MSE": torch.nn.MSELoss(),
                "LPIPS": LPIPSLoss(net_type="vgg", normalize=True),
                "SSIM": SSIMLoss(data_range=1.0),
                "PSNR": PSNRLoss(data_range=1.0)
            }
        else:
            raise ValueError(f"Unknown model architecture: {self.model.__class__.__name__}")
        
        return eval_metrics


    def evaluation_step(self, batch, verbose=False):
        losses = {}

        self.model.eval()
        with torch.no_grad():
            # decoder with reference
            if self.reference is not None:
                images, labels, reference_images, reference_labels  = batch
                if self.delta:
                    labels = labels - reference_labels

                outputs = self.model(reference_images, labels[:,self.dimensions_to_learn])
                for key, loss_fn in self.eval_loss_functions.items():
                    loss = loss_fn(outputs.clamp(0, 1), images.clamp(0,1)) # clamp reconstruction
                    losses[key] = loss.item()

            else:
                images, labels = batch

                # decide input and target based on model type
                if "encoder" in self.model_type:
                    outputs = self.model(images)
                    for key, loss_fn in self.eval_loss_functions.items():
                        loss = loss_fn(outputs, labels[:,self.dimensions_to_learn])

                        if key == "MSE per dimension":
                            # log each dimension separately
                            for dim_idx, dim_loss in enumerate(loss):
                                losses[f"MSE_dim_{DIMENSIONS[dim_idx]}"] = dim_loss.item()
                        else:
                            losses[key] = loss.item()

                elif "decoder" in self.model_type:
                    outputs = self.model(labels[:,self.dimensions_to_learn])
                    for key, loss_fn in self.eval_loss_functions.items():
                        loss = loss_fn(outputs, images)
                        losses[key] = loss.item()

                else:
                    raise ValueError(f"Unknown model type: {self.model_type}")

        mse_keys = []
        for key, loss in  losses.items():
            if "MSE" in key:
                mse_keys.append(key)
            if verbose:
                print(f"{key} loss: {loss:.4f}")
        # add RMSE for each MSE
        for key in mse_keys:
            losses["R"+key] = np.sqrt(losses[key])

        return outputs, losses

    def evaluate_model(self, test_loader: DataLoader, verbose=True, log=False):  
        self.model.eval()      
        test_loader, self.model, self.optimizer = self.accelerator.prepare(test_loader, self.model, self.optimizer)

        running_losses = {}
        for key in self.eval_loss_functions.keys():
            self.eval_loss_functions[key] = self.accelerator.prepare(self.eval_loss_functions[key])

        for batch in tqdm(test_loader, desc="Testing", total=len(test_loader), leave=False):
            outputs, losses = self.evaluation_step(batch, verbose=False)
            for key, loss in losses.items():
                if key not in running_losses:
                    running_losses[key] = 0.0
                running_losses[key] += losses[key]

        # average losses
        losses = {}
        for key in running_losses.keys():
            losses[key] = running_losses[key] / len(test_loader)
        # add RMSE for each MSE
        for key in losses.keys():
            if "MSE" in key and "RMSE" not in key:
                losses["R"+key] = np.sqrt(losses[key])
        
        if log:
            for key, loss in losses.items():
                self.logger.print(f"{key} loss: {loss:.4f}")
        elif verbose:
            for key, loss in losses.items():
                print(f"{key} loss: {loss:.4f}")
        
        return losses


    def encode_with_decoder(self, 
                            image, 
                            encoder=None, # encoder for initializing latent space config, otherwise random
                            references=None, # reference images for architectures with reference
                            threshold=0.001, 
                            max_iterations=100, 
                            verbose=False):
        """
        Generate images reconstructions with the decoder
        until the distance (LPIPS) between the input image and the reconstruction is below a certain threshold
        by searching the latent space with GD
        """
        if self.model.__class__.__name__ in DECODERS:
            raise ValueError("The model must be a decoder to use this method.")

        image = image.unsqueeze(0).to(self.accelerator.device)

        # Initialize latent/config vector (requires_grad=True for optimization) with encoder or random
        if encoder is not None:
            encoder.eval()
            with torch.no_grad():
                latent = encoder(image)
                latent.requires_grad = True
        else:
            latent = torch.randn(1, len(self.dimensions_to_learn),
                                    device=self.accelerator.device,
                                    requires_grad=True
                                )
        if verbose:
            print(f"Starting encoding with decoder, initial guess: {latent}")

        optimizer = torch.optim.Adam([latent])
        best_latent = latent.clone().detach()
        best_loss = float("inf")
        last_distance = None

        for i in range(max_iterations):
            if self.model.__class__.__name__ in ARCHITECTURES_WITH_REFERENCE:
                if references is None:
                    raise ValueError("Reference images must be provided for architectures with reference.")
                
                # select closest reference 
                closest_ref_img_idx = torch.min([self.eval_loss_functions["LPIPS"](ref_img, image) for ref_img in references])
                closest_ref_img = references[closest_ref_img_idx].unsqueeze(0)
                reconstructed_image = self.model(closest_ref_img, latent)
            else:
                reconstructed_image = self.model(latent)
            lpips_distance = self.eval_loss_functions["LPIPS"](reconstructed_image.clamp(0,1), image)

            optimizer.zero_grad()
            lpips_distance.backward()
            optimizer.step()

            if lpips_distance.item() < best_loss:
                best_loss = lpips_distance.item()
                best_latent = latent.clone().detach()

            if last_distance is not None and abs(last_distance - lpips_distance.item()) < threshold:
                if verbose:
                    print(f"Converged after {i+1} iterations, LPIPS={lpips_distance.item():.4f})")
                break
            last_distance = lpips_distance.item()

        if verbose and lpips_distance.item() >= threshold:
            print(f"Did not converge after {max_iterations} iterations, final best LPIPS={best_loss:.4f}")
        return best_latent, best_loss

