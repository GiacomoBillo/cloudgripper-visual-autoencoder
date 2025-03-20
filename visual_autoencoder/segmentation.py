import torch
import os
import cv2
import re
from data_loader import GripperDataset, store_image
from tqdm import tqdm
from torchvision import transforms

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


# segmentation
class MaskEngine():
    def __init__(self, environment):
        if not isinstance(environment, torch.Tensor):
            environment = torch.tensor(environment, device=DEVICE)
        self.environment = environment


    def create_mask(self, img):
        epsilon = 1e-8
        threshold = 1

        if not isinstance(img, torch.Tensor):
            img = torch.tensor(img)

        if len(img.shape) == 3: # single image
            # img = img.unsqueeze(0)
            sum_dim = 0
        else: # batch
            sum_dim = 1

        subtraction = torch.abs(torch.log(self.environment +epsilon) - torch.log(img +epsilon))
        # sum RGB values
        mask = torch.sum(subtraction, dim=sum_dim, keepdim=True)
        # threshold mask
        mask = (mask > threshold)
        return mask
    

    def apply_mask(self, img, mask):
        return img * mask
        # return np.where(mask, img, 0).astype(np.float32)


"""
generate and save masks and masked images
"""
if __name__ == "__main__":
    # dataset (access using either experiment or path)
    experiment = "data_collection_clean_env" # add the name of the dataset experiment HERE
    # path = # add the path of the dataset HERE
    sessions = [str(session) for session in range(1,21)] # first 20 sessions

    # load mean image
    transform = transforms.ToTensor()
    env_img_path_top = os.path.join("visual_autoencoder", "clean_environment_images", "top_mean_Experiment_data_collection_clean_env.jpeg")
    if os.path.exists(env_img_path_top):
        environment_img_top = transform(cv2.imread(env_img_path_top))
    else:
        raise FileNotFoundError(f"{env_img_path_top} does not exists")
    
    # create MaskEngine
    mask_engine_top = MaskEngine(environment_img_top)
    # mask_engine_bottom = MaskEngine(environment_img_bottom)


    for session in sessions:
        session_path = os.path.join(path, str(session), "task")

        dataset = GripperDataset(experiment=experiment, sessions=[session])
        # dataset = GripperDataset(abs_path=path, sessions=[session])

        for index in tqdm(range(len(dataset)), desc=f"Samples session {session}"):
            bottom_image, top_image, state = dataset[index]

            filename = dataset.top_images[index] # path/name_number.jpeg
            match = re.search(r'(\d+)', filename[::-1])
            # if match:
            number = match.group(1)[::-1]

            # top
            top_mask = mask_engine_top.create_mask(top_image)
            top_masked_image = mask_engine_top.apply_mask(top_image, top_mask)
            # print(top_mask.shape, top_mask.dtype)
            # print(top_masked_image.shape, top_masked_image.dtype)

            # bottom
            # bottom_mask = mask_engine_bottom.create_mask(bottom_image)
            # bottom_masked_image = mask_engine_bottom.apply_mask(bottom_image,bottom_mask)

            # save
            store_image(top_mask, os.path.join(session_path,"Top_masks", f"top_mask_{number}.jpeg"))
            store_image(top_masked_image, os.path.join(session_path,"Top_masked_images", f"top_masked_image_{number}.jpeg"))
            # store_image(bottom_mask, os.path.join(session_path,"Bottom_masks",f"bottom_mask_{number}.jpeg"))
            # store_image(bottom_masked_image, os.path.join(session_path,"Bottom_masked_images",f"bottom_masked_image_{number}.jpeg"))
