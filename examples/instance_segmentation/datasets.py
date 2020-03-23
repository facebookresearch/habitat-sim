import numpy as np
import torch
from common import area_filter, create_mask_filter
from torch.utils.data import Dataset


class InstanceSegmentationDataset(Dataset):
    def __init__(self, extractor, labels_we_care_about, transform=None):
        self.extractor = extractor
        self.transform = transform
        self.instance_id_to_name = extractor.instance_id_to_name
        self.mask_filter = create_mask_filter(labels_we_care_about, extractor)

        # Create a mapping from class name to class ID
        self.name_to_class_id = {
            name: id_val for id_val, name in enumerate(labels_we_care_about)
        }
        # And create the reverse mapping
        self.class_id_to_name = {
            id_val: name for name, id_val in self.name_to_class_id.items()
        }

    def __len__(self):
        return len(self.extractor)

    def __getitem__(self, idx):
        sample = self.extractor[idx]
        img, mask = sample["rgba"][:, :, :3], sample["semantic"]
        # scene_num = sample["scene_num"] # Instance Ids are different for each scene, so we need to know which scene this sample came from
        mask = self.mask_filter(mask)
        H, W = mask.shape
        instance_ids = np.unique(mask)

        # get bounding box coordinates, mask, and label for each instance_id
        masks = []
        labels = []
        boxes = []
        areas = []
        num_instances = len(instance_ids)
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
            if area_filter(cur_mask, box, H, W, size_tol=0.1):
                boxes.append(list(box))
                masks.append(cur_mask)
                # name = self.instance_id_to_name[(instance_ids[i], scene_num)]
                name = self.instance_id_to_name[instance_ids[i]]
                labels.append(self.name_to_class_id[name])
                areas.append((ymax - ymin) * (xmax - xmin))

        # convert everything into a torch.Tensor
        boxes = torch.as_tensor(boxes, dtype=torch.float32)
        labels = torch.as_tensor(labels, dtype=torch.int64)
        masks = torch.as_tensor(masks, dtype=torch.uint8)
        image_id = torch.tensor([idx])
        areas = torch.as_tensor(areas, dtype=torch.float32)
        # suppose all instances are not crowd
        iscrowd = torch.zeros((num_instances,), dtype=torch.int64)

        target = {}
        target["boxes"] = boxes
        target["labels"] = labels
        target["masks"] = masks
        target["image_id"] = image_id
        target["area"] = areas
        target["iscrowd"] = iscrowd

        if self.transform:
            img = self.transform(img)

        return img, target
