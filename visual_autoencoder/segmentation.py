import torch
import os
import cv2
import re
from gripper_data import GripperDataset, store_image
from tqdm import tqdm
from torchvision import transforms
from background_subtraction import morphological_refinement, filter_small_components_reverse, get_segmentation_mask, filter_small_components, create_mask_image, unite_masks


DEVICE = "cuda" if torch.cuda.is_available() else "cpu"


# segmentation
class MaskEngine():
    def __init__(self, environment):
        if not isinstance(environment, torch.Tensor):
            environment = torch.tensor(environment, device=DEVICE)
        self.environment = environment


    def create_mask(self, img, post_process=True):
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

        # post processing to clean the mask
        if post_process:
            if len(img.shape) == 3: # single image
                mask = self.post_process_mask(mask.squeeze())
            else: # batch
                for i in range(mask.shape[0]):
                    mask[i] = self.post_process_mask(mask[i].squeeze())
            
        return mask
    

    def post_process_mask(self, mask):
        # mask = morphological_refinement(mask.squeeze(0))
        mask = filter_small_components_reverse(mask.squeeze())

        segmentation_masks = get_segmentation_mask(mask)
        mask = segmentation_masks[0] # select biggest mask component

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
    path = os.getenv("DATASET_PATH") # add the path of the dataset HERE
    sessions = [str(session) for session in range(1,2)] # first 20 sessions

    # load mean image
    transform = transforms.ToTensor()
    env_img_path_top = os.path.join("visual_autoencoder", "clean_environment_images", "top_mean.jpeg")
    if os.path.exists(env_img_path_top):
        environment_img_top = transform(cv2.imread(env_img_path_top))
    else:
        raise FileNotFoundError(f"{env_img_path_top} does not exists")
    
    # create MaskEngine
    mask_engine_top = MaskEngine(environment_img_top)
    # mask_engine_bottom = MaskEngine(environment_img_bottom)


    for session in sessions:
        session_path = os.path.join(path, str(session), "task")

        # dataset = GripperDataset(experiment=experiment, sessions=[session])
        dataset = GripperDataset(abs_path=path, sessions=[session], 
                                 images_to_load=["Images"])

        for index in tqdm(range(len(dataset)), desc=f"Samples session {session}"):
            image, state = dataset[index]

            filename = dataset.images["Images"][index] # path/name_number.jpeg
            match = re.search(r'(\d+)', filename[::-1])
            # if match:
            number = match.group(1)[::-1]

            # top
            mask = mask_engine_top.create_mask(image)
            masked_image = mask_engine_top.apply_mask(image, mask)
            # print(mask.shape, mask.dtype)
            # print(masked_image.shape, masked_image.dtype)

            # save
            store_image(mask.squeeze(), os.path.join(session_path,
                                                     "Top_masks_processed", 
                                                     f"top_mask_{number}.jpeg"))
            store_image(masked_image, os.path.join(session_path,
                                                   "Top_masked_images_processed", 
                                                   f"top_masked_image_{number}.jpeg"))