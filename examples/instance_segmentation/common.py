import collections
import random

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
import torch
from scipy import stats


def create_mask_filter(labels, extractor):
    instance_id_to_name = extractor.instance_id_to_name
    labels_we_care_about = set(
        [
            instance_id
            for instance_id, name in instance_id_to_name.items()
            if name in labels
        ]
    )

    # Function that filters out instance of objects we do not care about
    mask_filter = np.vectorize(lambda x: x * int(x in labels_we_care_about))
    return mask_filter


def area_filter(mask, bounding_box, img_height, img_width, size_tol=0.05):
    """
    Function to filter out masks that contain sparse instances
    for example:

        0 0 0 0 0 0
        1 0 0 0 0 0
        1 0 0 0 1 0    This is a sparse mask
        0 0 0 0 1 0
        0 0 0 0 0 0


        0 0 0 0 0 0
        1 1 1 1 1 0
        1 1 1 1 1 1    This is not a sparse mask
        0 0 0 1 1 1
        0 0 0 0 0 0
    """
    xmin, ymin, xmax, ymax = bounding_box
    num_positive_pixels = np.sum(mask[ymin:ymax, xmin:xmax])
    num_total_pixels = (xmax - xmin) * (ymax - ymin)
    not_sparse = num_positive_pixels / num_total_pixels >= 0.3
    big_enough = (xmax - xmin) >= size_tol * img_width and (
        ymax - ymin
    ) >= size_tol * img_height
    return not_sparse and big_enough


class InstanceVisualizer:
    def __init__(self, model, dataloader, device, num_classes):
        self.model = model
        self.dataloader = dataloader
        self.device = device
        self.num_classes = num_classes

        num_unique_instance_colors = 3
        self.instance_cmap = {
            instance_class: [
                random.randint(1, 1000) for _ in range(num_unique_instance_colors)
            ]
            for instance_class in range(self.num_classes)
        }
        self.instance_cmap[0] = [
            0,
            0,
            0,
        ]  # Make sure background (class 0) stays all the same color

    def visualize_instance_segmentation_output(
        self, max_num_outputs=float("inf"), segment_type="instance"
    ):
        def visual_filter(masks, labels, segment_type):
            masks = masks.astype(int)
            H, W = masks.shape[2:]

            # Instead of T/F for each mask, use Class_label/0
            visual = np.zeros((H, W))
            for i, mask in enumerate(masks):
                np.putmask(masks[i], mask, labels[i])

            visuals = self._compute_mode_ignore_zeros(masks)
            return visuals

        assert segment_type in ["instance", "semantic"]
        self.model.eval()
        with torch.no_grad():
            cpu_device = torch.device("cpu")
            visuals = {}
            num_out_so_far = 1
            for image, targets in self.dataloader:
                image = list(img.to(self.device) for img in image)
                targets = [
                    {k: v.to(self.device) for k, v in t.items()} for t in targets
                ]

                torch.cuda.synchronize()
                outputs = self.model(image)
                outputs = [{k: v.to(cpu_device) for k, v in t.items()} for t in outputs]
                res = {
                    target["image_id"].item(): output
                    for target, output in zip(targets, outputs)
                }
                for image_id in res.keys():
                    masks = res[image_id]["masks"].numpy() >= 0.5
                    labels = res[image_id]["labels"].numpy()
                    visual = visual_filter(masks, labels, segment_type)
                    visuals[image_id] = visual

                if num_out_so_far == max_num_outputs:
                    break

                num_out_so_far += 1

        return visuals

    def _compute_mode_ignore_zeros(self, masks):
        H, W = masks.shape[2:]
        modes = np.zeros((H, W))
        for i in range(H):
            for j in range(W):
                pixels = masks[:, 0, i, j]
                counts = collections.defaultdict(int)
                max_label = 0
                max_count = 0
                for p in pixels:
                    if p != 0:
                        counts[p] = counts[p] + 1
                        if counts[p] > max_count:
                            max_count = counts[p]
                            max_label = p

                modes[i, j] = max_label

        return modes
