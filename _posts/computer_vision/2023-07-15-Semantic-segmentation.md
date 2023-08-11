---
layout: post
title: "Part 01 : Training a Semantic segmentation model"
date: 2023-07-15 20:22:00-0500
description: 
# youtubeId: 90K8CWjifUs
categories: Computer-Vision
---

This briefly gives an overview of how a neural network could be trainded to perform semantic segmentation. I use the [Cityscapes dataset](https://www.cityscapes-dataset.com/login/). You would require a login id and password to download the dataset. Once you obtain this, download the `gtFine_trainvaltest.zip` and the `leftImg8bit_trainvaltest.zip`. Setup your virtual environment with 
* `numpy`, `Pillow`, `albumentations`, `typing_extensions`, `torchvision==0.15.2`, `torch==2.0.1`, `segmentation-models-pytorch`, `torchmetrics`, `opencv-python`, `matplotlib`


I have created a utility function python file which does all the repeated tasks and it is as follows

```bash
import torch
import numpy as np
from PIL import Image
import albumentations as A
from typing import Any, Tuple
import torchvision.transforms as T
from torchvision.datasets import Cityscapes
from albumentations.pytorch import ToTensorV2


def save_checkpoint(epoch, model, optimizer, name="chkpt.pth.tar"):
    """
    Save model checkpoint.

    :param epoch: epoch number
    :param model: model
    :param optimizer: optimizer
    :param name: name of the saved model
    """
    state = {'epoch': epoch,
             'model': model,
             'optimizer': optimizer}
    torch.save(state, name)


def writeToFile(content, filename="metrics.txt", mode="w" ):
    with open(filename, mode) as f:
        f.write(str(content))
    f.close()

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

inv_normalize = T.Normalize(
    mean=[-0.485 / 0.229, -0.456 / 0.224, -0.406 / 0.225],
    std=[1 / 0.229, 1 / 0.224, 1 / 0.255]
)

ignore_index = 255
void_classes = [0, 1, 2, 3, 4, 5, 6, 9, 10, 14, 15, 16, 18, 29, 30, -1]
valid_classes = [ignore_index, 7, 8, 11, 12, 13, 17, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 31, 32, 33]
class_names = ['unlabelled', 'road', 'sidewalk', 'building', 'wall', 'fence', 'pole', 'traffic_light', \
               'traffic_sign', 'vegetation', 'terrain', 'sky', 'person', 'rider', 'car', 'truck', 'bus', \
               'train', 'motorcycle', 'bicycle']
#why i choose 20 classes
#https://stackoverflow.com/a/64242989

class_map = dict(zip(valid_classes, range(len(valid_classes))))
num_of_classes = len(valid_classes)

colors = [[0,   0,   0],
        [128, 64, 128],
        [244, 35, 232],
        [70, 70, 70],
        [102, 102, 156],
        [190, 153, 153],
        [153, 153, 153],
        [250, 170, 30],
        [220, 220, 0],
        [107, 142, 35],
        [152, 251, 152],
        [0, 130, 180],
        [220, 20, 60],
        [255, 0, 0],
        [0, 0, 142],
        [0, 0, 70],
        [0, 60, 100],
        [0, 80, 100],
        [0, 0, 230],
        [119, 11, 32],
    ]

label_colours = dict(zip(range(num_of_classes), colors))

transform = A.Compose(
[
    # A.Resize(224, 224),
    A.Resize(256, 512),
    A.HorizontalFlip(),
    A.Normalize(mean=(0.485, 0.456, 0.406), std=(0.229, 0.224, 0.225)),
    ToTensorV2(),
]
)


def encode_segmap(mask):
    #remove unwanted classes and recitify the labels of wanted classes
    for _voidc in void_classes:
        mask[mask == _voidc] = ignore_index
    for _validc in valid_classes:
        mask[mask == _validc] = class_map[_validc]
    return mask


def decode_segmap(temp):
    #convert gray scale to color
    a = label_colours.copy()
    temp = temp.numpy()
    r = temp.copy()
    g = temp.copy()
    b = temp.copy()
    for l in range(0, num_of_classes):
        r[temp == l] = label_colours[l][0]
        g[temp == l] = label_colours[l][1]
        b[temp == l] = label_colours[l][2]

    rr, gg, bb = temp.copy(), temp.copy(), temp.copy()
    for l in range(num_of_classes):
        for i in range(temp.shape[0]):
            for j in range(temp.shape[1]):
                if temp[i, j] == l:
                    rr[i, j] = label_colours[l][0]
                    gg[i, j] = label_colours[l][1]
                    bb[i, j] = label_colours[l][2]

    rgb = np.zeros((temp.shape[0], temp.shape[1], 3))
    rgb[:, :, 0] = r / 255.0
    rgb[:, :, 1] = g / 255.0
    rgb[:, :, 2] = b / 255.0
    return rgb


class MyClass(Cityscapes):
    def __getitem__(self, index: int) -> Tuple[Any, Any]:
        image = Image.open(self.images[index]).convert('RGB')

        targets: Any = []
        for i, t in enumerate(self.target_type):
            if t == 'polygon':
                target = self._load_json(self.targets[index][i])
            else:
                target = Image.open(self.targets[index][i])
            targets.append(target)
        target = tuple(targets) if len(targets) > 1 else targets[0]

        if self.transforms is not None:
            transformed = transform(image=np.array(image), mask=np.array(target))
        return transformed['image'], transformed['mask']
```

The training of the model is carried out by using models from `segmentation_models_pytorch` as this is an easy to use library for training several models. The training script is as follows

```bash
import torch
import torchmetrics
import utils_fn as utl
import multiprocessing
from torch.utils.data import DataLoader
import segmentation_models_pytorch as smp

transform = utl.transform
numworker = multiprocessing.cpu_count() // 4
batch_size = 32

trainset = utl.MyClass('../../datasets/cityscapes/', split='train', mode='fine', target_type='semantic', transforms=utl.transform)
valset = utl.MyClass('../../datasets/cityscapes/', split='val', mode='fine', target_type='semantic', transforms=utl.transform)
trainloader = DataLoader(trainset, batch_size=batch_size,  shuffle=True, num_workers=numworker, pin_memory=True)
valloader = DataLoader(valset, batch_size=batch_size,  shuffle=False, num_workers=numworker, pin_memory=True)

# https://github.com/qubvel/segmentation_models.pytorch#models
model = smp.Unet(
        encoder_name="timm-mobilenetv3_small_minimal_100", # choose encoder, e.g. mobilenet_v2 or efficientnet-b7
        encoder_weights="imagenet",     # use `imagenet` pre-trained weights for encoder initialization
        in_channels=3,                  # model input channels (1 for gray-scale images, 3 for RGB, etc.)
        classes=utl.num_of_classes,
        )

model = model.to(utl.device)


lr = 0.001
momentum = 0.9
weightDecay = 0.005

params = [p for p in model.parameters() if p.requires_grad]
optimizer = torch.optim.AdamW(params, lr=lr)
metrics = torchmetrics.JaccardIndex(num_classes=utl.num_of_classes, task="multiclass")
metrics1 = torchmetrics.classification.Accuracy(num_classes=utl.num_of_classes, task="multiclass")

criterion = smp.losses.DiceLoss(mode='multiclass')

dataloader = {"train": trainloader, "val": valloader}


def train_model(model, dataloader, criterion, optimizer, num_epochs, load_chkpt=False, path=None):
    for epoch in range(num_epochs):
        print(f"Epoch: {epoch}/{num_epochs}")
        train_loss = 0.0
        model.train()
        ii = 0
        for inputs, labels in trainloader:
            if ii % 100 == 0:
                print(ii)
            inputs = inputs.to(utl.device)
            labels = labels.to(utl.device).long()
            optimizer.zero_grad()
            predictions = model.forward(inputs)
            target = utl.encode_segmap(labels)
            loss = criterion(predictions, target)
            acc = metrics1(predictions, target)
            loss.backward()
            optimizer.step()
            train_loss += loss.item()
            ii += 1

        valid_loss = 0.0
        model.eval()  # Optional when not using Model Specific layer
        for inputs, labels in valloader:
            inputs = inputs.to(utl.device)
            labels = labels.to(utl.device).long()
            predictions = model.forward(inputs)
            target = utl.encode_segmap(labels)
            loss = criterion(predictions, target)
            valid_loss = loss.item() * inputs.size(0)

        print(f'Epoch {epoch + 1} \t\t Training Loss: {train_loss / len(trainloader)} \t\t Validation Loss: {valid_loss / len(valloader)}')
        if not epoch % 20 and epoch != num_epochs:  # save model every 20 epoch
            print("Saving model")
            utl.save_checkpoint(epoch=epoch, model=model, optimizer=optimizer,
                                name="chk_pts/trained_cityscapes_" + model.name + "_" + str(epoch) + ".pth")
        if epoch == num_epochs - 1:
            utl.save_checkpoint(epoch=epoch, model=model, optimizer=optimizer,
                                name="chk_pts/trained_cityscapes_final_" + model.name + ".pth")
            acc = metrics1.compute()
            content = {"Accuracy": acc.item(), "Epochs": epoch+1, "ModelName": model.name, "Train_loss": train_loss, "Valid_loss": valid_loss}
            utl.writeToFile(content=content, filename=model.name + "_metrics.txt", mode="w")

chkpt_path = "chk_pts/trained_cityscapes.pth"
train_model(model=model, dataloader=dataloader, criterion=criterion, optimizer=optimizer, num_epochs=10,
            load_chkpt=False, path=chkpt_path)
print("done")
```

### A few things to note
* Here `../../datasets/cityscapes/` is the folder where the `gtFine` and `leftImg8bit` folders are present. Both these folders are inside the downloaded and extracted `gtFine_trainvaltest.zip` and the `leftImg8bit_trainvaltest.zip` respectively. 
* Albumentations is much faster than the torchvision.transforms library
* Play aroud with the model as explained in the [segmentations_model_pytorch github](https://github.com/qubvel/segmentation_models.pytorch#models)


