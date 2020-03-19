import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
import torchvision.transforms as T
from torch.utils.data import Dataset

import habitat_sim
from habitat_sim.utils.data import ImageExtractor

from .engine import load_model_state, save_model_state, train_one_epoch
from .models import build_maskrcnn_model


class TrainingEnvironment:
    def __init__(self, num_workers=1):
        self.num_workers = num_workers

    def train(self):
        raise NotImplementedError

    def test(self):
        raise NotImplementedError


class InstanceSegmentationEnvironment(TrainingEnvironment):
    def __init__(self, scene, learning_rate=0.00005, momentum=0.9, weight_decay=0.0005):
        self.scene = scene
        self.extractor = ImageExtractor(scene=scene)
        self.classes = self.extractor.get_semantic_class_names()
        self.num_classes = len(self.classes)
        self.model = build_maskrcnn_model(self.num_classes)

        # Specify which transforms to apply to the data in preprocessing
        self.transforms = T.Compose([T.ToTensor()])
        self.train_dataset = HabitatDataset(self.extractor, transform=self.transforms)
        self.train_dataloader = torch.utils.data.DataLoader(
            dataset, batch_size=2, shuffle=True, collate_fn=collate_fn
        )

        self.model_weight_path = ""
        self.device = (
            torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        )
        self.model.to(self.device)
        params = [p for p in model.parameters() if p.requires_grad]
        self.optimizer = torch.optim.SGD(
            params, lr=learning_rate, momentum=momentum, weight_decay=weight_decay
        )
        self.lr_scheduler = torch.optim.lr_scheduler.StepLR(
            optimizer, step_size=3, gamma=0.1
        )

    def train(
        self, num_epochs=100, load_state=False, model_weight_path=None, save_freq=20
    ):
        epoch = 0
        if load_state and model_weight_path is not None:
            epoch = load_model_state(self.model, self.optimizer, model_weight_path)

        self.extractor.set_mode("train")
        for _ in range(num_epochs):
            train_one_epoch(
                self.model,
                self.optimizer,
                self.train_dataloader,
                self.device,
                epoch,
                print_freq=10,
            )

            self.lr_scheduler.step()
            epoch += 1

            if epoch % save_freq == 0:
                print("\nSaving model weights!\n")

    def test(self):
        pass


def collate_fn(batch):
    return tuple(zip(*batch))


class InstanceSegmentationDataset(Dataset):
    def __init__(self, extractor, transform=None):
        self.extractor = extractor
        self.transform = transform

    def __len__(self):
        return len(self.extractor)

    def __getitem__(self, idx):
        sample = self.extractor[idx]
        img, mask = sample["rgba"][:, :, :3], sample["semantic"]
        H, W = mask.shape
        instance_ids = np.unique(mask)

        # get bounding box coordinates, mask, and label for each instance_id
        masks = []
        labels = []
        boxes = []
        areas = []
        num_instances = len(instance_ids)

        # There are much more efficient ways to create the data involving caching and
        # preprocessing but efficiency is not the focus of this example
        for i in range(num_instances):
            cur_mask = mask == instance_ids[i]
            pos = np.where(cur_mask)
            xmin = np.min(pos[1])
            xmax = np.max(pos[1])
            ymin = np.min(pos[0])
            ymax = np.max(pos[0])

            # Avoid zero area boxes
            if xmin == xmax:
                xmin = max(0, xmin - 1)
                xmax = min(W, xmax + 1)
            if ymin == ymax:
                ymin = max(0, ymin - 1)
                ymax = min(H, ymax + 1)

            box = (xmin, ymin, xmax, ymax)
            boxes.append(list(box))
            masks.append(cur_mask)
            name = "hi"
            labels.append(1)
            areas.append((ymax - ymin) * (xmax - xmin))

        # convert everything into a torch.Tensor
        boxes = torch.as_tensor(boxes, dtype=torch.float32)
        # there is only one class
        labels = torch.ones((num_instances,), dtype=torch.int64)
        masks = torch.as_tensor(masks, dtype=torch.uint8)
        image_id = torch.tensor([idx])
        area = (boxes[:, 3] - boxes[:, 1]) * (boxes[:, 2] - boxes[:, 0])
        # suppose all instances are not crowd
        iscrowd = torch.zeros((num_instances,), dtype=torch.int64)
        target = {}
        target["boxes"] = boxes
        target["labels"] = labels
        target["masks"] = masks
        target["image_id"] = image_id
        target["area"] = area
        target["iscrowd"] = iscrowd

        if self.transform:
            img = self.transform(img)

        return img, target
