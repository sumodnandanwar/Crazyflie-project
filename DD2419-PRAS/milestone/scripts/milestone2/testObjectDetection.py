import os
import torch
import numpy as np
import torch.utils.data
from PIL import Image
import utils
import transforms as T
import cv2

import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
 
import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor
import matplotlib.pyplot as plt
import matplotlib.patches as patches

def get_transform(train):
    transforms = []
    # converts the image, a PIL image, into a PyTorch Tensor
    transforms.append(T.ToTensor())
    if train:
        # during training, randomly flip the training images
        # and ground-truth for data augmentation
        transforms.append(T.RandomHorizontalFlip(0.5))
 
    return T.Compose(transforms)

def test_object(img_raw):
    transform = get_transform(train=False)
    target = {}
    target["boxes"] = torch.tensor([0,0,0,0])
    # target["labels"] = torch.tensor([0])

    img,_= transform(img_raw,target)

    # put the model in evaluation mode
    model.eval()
    with torch.no_grad():
        prediction = model([img.to(device)])

    pred = prediction[0]
    res2 = pred['boxes'][pred["scores"]>0.7].cpu().numpy()
    label = pred["labels"][pred["scores"]>0.7].cpu().numpy()

    return res2,label

    # # Create figure and axes
    # fig,ax = plt.subplots(1)

    # # Display the image
    # ax.imshow(img_raw)

    # # Create a Rectangle patch
    # for res in res2:
    #     rect = patches.Rectangle((res[0],res[1]),res[2]-res[0],res[3]-res[1],linewidth=1,edgecolor='r',facecolor='none')

    #     # Add the patch to the Axes
    #     ax.add_patch(rect)

    # plt.show()

    



device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

model = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)

num_classes = 3
in_features = model.roi_heads.box_predictor.cls_score.in_features

model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)


model.to(device)

model.load_state_dict(torch.load("model_traffic"))

# if __name__ == '__main__':
#     img = cv2.imread("cv_image.jpg")
#     test_object(img)
    

# index = 35
# for index in range(30,60):
#     img_raw = cv2.imread("saveImg/testDataset/img"+str(index)+".jpg")
#     if np.shape(img_raw) != (): 
#         transform = get_transform(train=False)
#         target = {}
#         target["boxes"] = torch.tensor([0,0,0,0])
#         # target["labels"] = torch.tensor([0])

#         img,_= transform(img_raw,target)

#         # put the model in evaluation mode
#         model.eval()
#         with torch.no_grad():
#             prediction = model([img.to(device)])



#         pred = prediction[0]
#         res2 = pred['boxes'][pred["scores"]>0.7].cpu().numpy()


#         # Create figure and axes
#         fig,ax = plt.subplots(1)

#         # Display the image
#         ax.imshow(img_raw)

#         # Create a Rectangle patch
#         for res in res2:
#             rect = patches.Rectangle((res[0],res[1]),res[2]-res[0],res[3]-res[1],linewidth=1,edgecolor='r',facecolor='none')

#             # Add the patch to the Axes
#             ax.add_patch(rect)
#         plt.savefig("saveImg/testDataset/res"+str(index)+".jpg")
#         plt.close()

    # plt.show()


