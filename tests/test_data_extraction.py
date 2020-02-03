# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from habitat_sim.utils.data.dataextractor import HabitatDataset, TopdownView


class TrivialNet(nn.Module):
    def __init__(self):
        super(TrivialNet, self).__init__()
        self.conv1 = nn.Conv2d(
            4, 4, kernel_size=10, stride=10
        )  # Conv to reduce memory for params
        self.out = nn.Linear(10404, 10)

    def forward(self, x):
        x = self.conv1(x)
        flat_dim_size = self.get_flat_dim(x)
        x = x.view(-1, flat_dim_size)
        x = self.out(x)
        return x

    def get_flat_dim(self, x):
        size = 1
        for dim in x.size()[1:]:
            size *= dim

        return size


def test_topdown_view(sim):
    tdv = TopdownView(sim, res=0.1)
    topdown_view = tdv.topdown_view


def test_data_extractor_end_to_end(sim):
    # Path is relative to simulator.py
    scene_filepath = (
        "../../data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    )
    dataset = HabitatDataset(scene_filepath, labels=[0.0], img_size=(512, 512), sim=sim)
    dataloader = DataLoader(dataset, batch_size=3)

    net = TrivialNet()

    # Run data through network
    for i, sample_batch in enumerate(dataloader):
        img, label = sample_batch["rgb"], sample_batch["label"]
        img = img.permute(0, 3, 2, 1).float()  # Conv2d requires NCHW format
        out = net(img)
