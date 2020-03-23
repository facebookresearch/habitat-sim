import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
import torchvision.transforms as T
from common import InstanceVisualizer
from datasets import InstanceSegmentationDataset
from engine import evaluate, load_model_state, save_model_state, train_one_epoch
from models import build_maskrcnn_model

import habitat_sim
from habitat_sim.utils.data import ImageExtractor


def collate_fn(batch):
    return tuple(zip(*batch))


class TrainingEnvironment:
    def __init__(self, num_workers=1):
        self.num_workers = num_workers

    def train(self):
        raise NotImplementedError

    def test(self):
        raise NotImplementedError


class InstanceSegmentationEnvironment(TrainingEnvironment):
    def __init__(
        self, scene, lr=0.00005, momentum=0.9, weight_decay=0.0005, batch_size=8
    ):
        self.scene = scene
        self.extractor = ImageExtractor(
            scene_filepath=scene, output=["rgba", "semantic"]
        )
        self.classes = self.extractor.get_semantic_class_names()
        # labels = ['background'] + [name for name in labels if name not in ['background', 'void', '', 'objects']]
        self.num_classes = len(self.classes)
        self.model = build_maskrcnn_model(self.num_classes)

        # Specify which transforms to apply to the data in preprocessing
        self.transforms = T.Compose([T.ToTensor()])
        self.dataset = InstanceSegmentationDataset(
            self.extractor, self.classes, transform=self.transforms
        )
        self.dataloader = torch.utils.data.DataLoader(
            self.dataset, batch_size=batch_size, shuffle=True, collate_fn=collate_fn
        )

        self.device = (
            torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        )
        self.model.to(self.device)
        params = [p for p in self.model.parameters() if p.requires_grad]
        self.optimizer = torch.optim.SGD(
            params, lr=lr, momentum=momentum, weight_decay=weight_decay
        )
        self.lr_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
            self.optimizer, verbose=True, patience=4
        )
        # self.writer = SummaryWriter("examples/instance_segmentation/runs/")

    def train(
        self, num_epochs=100, load_path=None, save_path="saved-weights.pt", save_freq=20
    ):
        epoch = 1
        if load_path is not None:
            epoch = load_model_state(self.model, self.optimizer, load_path)

        for _ in range(num_epochs):
            self.extractor.set_mode("train")
            train_one_epoch(
                model=self.model,
                optimizer=self.optimizer,
                data_loader=self.dataloader,
                device=self.device,
                epoch=epoch,
                print_freq=10,
                lr_scheduler=self.lr_scheduler,
            )

            epoch += 1
            if epoch % 5 == 0:
                save_model_state(self.model, self.optimizer, epoch, save_path)
                self.extractor.set_mode("test")
                evaluate(self.model, self.dataloader, device=self.device)

    def test(self):
        # Put the extractor into test mode so that it will use the test data
        self.extractor.set_mode("test")
        evaluator = evaluate(self.model, self.dataloader, device=self.device)

    def visualize(self, mode="train", num_batches=1):
        self.extractor.set_mode(mode)
        visualizer = InstanceVisualizer(
            self.model,
            self.dataloader,
            device=self.device,
            num_classes=self.num_classes,
        )
        visuals = visualizer.visualize_instance_segmentation_output(
            max_num_outputs=num_batches
        )
        for idx, visual in visuals.items():
            plt.imsave(f"img_{idx}.png", visual)
