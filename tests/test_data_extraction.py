# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset

from habitat_sim.utils.data.data_extractor import ImageExtractor, TopdownView


class TrivialNet(nn.Module):
    def __init__(self):
        super(TrivialNet, self).__init__()

    def forward(self, x):
        x = F.relu(x)
        return x


class MyDataset(Dataset):
    def __init__(self, extractor):
        self.extractor = extractor

    def __len__(self):
        return len(self.extractor)

    def __getitem__(self, idx):
        return self.extractor[idx]


def test_topdown_view(sim):
    tdv = TopdownView(sim, height=0.0, pixels_per_meter=0.1)
    topdown_view = tdv.topdown_view


def test_data_extractor_end_to_end(sim):
    # Path is relative to simulator.py
    scene_filepath = ""
    extractor = ImageExtractor(scene_filepath, labels=[0.0], img_size=(32, 32), sim=sim)
    dataset = MyDataset(extractor)
    dataloader = DataLoader(dataset, batch_size=3)
    net = TrivialNet()

    # Run data through network
    for i, sample_batch in enumerate(dataloader):
        img, label = sample_batch["rgba"], sample_batch["label"]
        img = img.permute(0, 3, 2, 1).float()
        out = net(img)
