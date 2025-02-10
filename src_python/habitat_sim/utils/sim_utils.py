# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

from typing import Dict, List

import numpy as np
from sklearn.cluster import DBSCAN

import habitat_sim


def get_floor_navigable_extents(
    hsim: habitat_sim.Simulator, num_points_to_sample: int = 20000
) -> List[Dict[str, float]]:
    """
    Function to estimate the number of floors in a 3D scene and the Y extents
    of the navigable space on each floor. It samples a random number
    of navigable points and clusters them based on their height coordinate.
    Each cluster corresponds to a floor, and the points within a cluster
    determine the extents of the navigable surfaces in a floor.
    """
    # randomly sample navigable points
    random_navigable_points = []
    for _i in range(num_points_to_sample):
        point = hsim.pathfinder.get_random_navigable_point()
        if np.isnan(point).any() or np.isinf(point).any():
            continue
        random_navigable_points.append(point)
    random_navigable_points = np.array(random_navigable_points)
    # cluster the rounded y_coordinates using DBScan
    y_coors = np.around(random_navigable_points[:, 1], decimals=1)
    clustering = DBSCAN(eps=0.2, min_samples=500).fit(y_coors[:, np.newaxis])
    c_labels = clustering.labels_
    n_clusters = len(set(c_labels)) - (1 if -1 in c_labels else 0)
    # estimate floor extents
    floor_extents = []
    core_sample_y = y_coors[clustering.core_sample_indices_]
    core_sample_labels = c_labels[clustering.core_sample_indices_]
    for i in range(n_clusters):
        floor_min = core_sample_y[core_sample_labels == i].min().item()
        floor_max = core_sample_y[core_sample_labels == i].max().item()
        floor_mean = core_sample_y[core_sample_labels == i].mean().item()
        floor_extents.append({"min": floor_min, "max": floor_max, "mean": floor_mean})

    # TODO: test this function
    return floor_extents
