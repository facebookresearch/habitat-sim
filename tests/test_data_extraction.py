# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import pytest

# skip all tests if torch not installed
torch = pytest.importorskip("torch")
from torch import nn as nn
from torch.nn import functional as F
from torch.utils.data import DataLoader, Dataset

import habitat_sim
from habitat_sim.utils.data.data_extractor import ImageExtractor
from habitat_sim.utils.data.data_structures import ExtractorLRUCache
from habitat_sim.utils.data.pose_extractor import TopdownView
from habitat_sim.utils.settings import make_cfg


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
        sample = self.extractor[idx]
        sample["label"] = 0
        return sample


def test_topdown_view(make_cfg_settings):
    with habitat_sim.Simulator(make_cfg(make_cfg_settings)) as sim:
        tdv = TopdownView(sim, height=0.0, meters_per_pixel=0.1)
        topdown_view = tdv.topdown_view
        assert type(topdown_view) == np.ndarray


def test_data_extractor_end_to_end(make_cfg_settings):
    # Path is relative to simulator.py
    with habitat_sim.Simulator(make_cfg(make_cfg_settings)) as sim:
        scene_filepath = ""
        extractor = ImageExtractor(
            scene_filepath, labels=[0.0], img_size=(32, 32), sim=sim
        )
        dataset = MyDataset(extractor)
        dataloader = DataLoader(dataset, batch_size=3)
        net = TrivialNet()

        # Run data through network
        for sample_batch in dataloader:
            img, _ = sample_batch["rgba"], sample_batch["label"]
            img = img.permute(0, 3, 2, 1).float()
            net(img)


def test_extractor_cache():
    cache = ExtractorLRUCache()
    cache.add(1, "one")
    cache.add(2, "two")
    cache.add(3, "three")
    assert cache[next(reversed(list(cache._order)))] == "three"
    accessed_data = cache[2]  # noqa: F841
    assert cache[next(reversed(list(cache._order)))] == "two"
    cache.remove_from_back()
    assert 1 not in cache


def test_pose_extractors(make_cfg_settings):
    with habitat_sim.Simulator(make_cfg(make_cfg_settings)) as sim:
        scene_filepath = ""
        pose_extractor_names = ["closest_point_extractor", "panorama_extractor"]
        for name in pose_extractor_names:
            extractor = ImageExtractor(
                scene_filepath, img_size=(32, 32), sim=sim, pose_extractor_name=name
            )
            assert len(extractor) > 1
