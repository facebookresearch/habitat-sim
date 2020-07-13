Habitat Sim Image Extractor Tutorial
####################################

.. contents::
    :class: m-block m-default


This notebook will go over how to use the Image Extraction API in Habitat Sim and the different user options available.

.. code:: py

    import os

    import matplotlib.pyplot as plt
    import numpy as np
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    from torch.utils.data import DataLoader, Dataset

    from habitat_sim.utils.data.data_extractor import ImageExtractor


    # Helper functions
    def display_sample(sample):
        img = sample["rgba"]
        depth = sample["depth"]
        semantic = sample["semantic"]
        label = sample["label"]

        arr = [img, depth, semantic]
        titles = ["rgba", "depth", "semantic"]
        plt.figure(figsize=(12, 8))
        for i, data in enumerate(arr):
            ax = plt.subplot(1, 3, i + 1)
            ax.axis("off")
            ax.set_title(titles[i])
            plt.imshow(data)

        plt.show()

`Setting up the Extractor`_
===========================

The main class that handles image data extraction in Habitat Sim is called ImageExtractor.
The user needs to provide a scene filepath (either a .glb or .ply file) to the constructor.
This is the only required constructor argument.

Required Constructor Args:

* scene_filepath (Str, List[Str]): The filepath to the scene file as explained above, or a list of filepaths.

Optional Args:

* labels (List): Class labels of the type of images the user wants to extract. Currently we only.
    support extracting images of 'unnavigable points' like walls. In the future we hope to extend this functionality to allow the user to specify more unique class labels, but for now this argument is not that useful.
* img_size (Tuple): The size of images to be output in the format (height, width).
* output (List): A list of the different output image types the user can obtain. The options are any of 'rgba', 'depth', and 'semantic'. Default is rgba.
* shuffle (Boolean): Whether or not to shuffle the extracted images.
* use_caching (Boolean): Whether or not to cache (up to capacity) images in memory rather than generating them from the simulator on the fly.
* meters_per_pixel (Float): Granularity of the topdown view used for setting the camera positions in the pose extractor.


Habitat Sim does not currently support multiple instances of extractors, so if you're done using
an extractor you need to call the close method before instantiating a new one. Below we will go
over some optional arguments for the extractor class.


`Using the Extractor`_
======================

The extractor can be indexed and sliced like a normal python list. Internaly, indexing into
the extractor sets an agent position and rotation within the simulator and returns the corresponding
agent observation. Indexing returns a dictionary that contains an image of each type specified in
the "output" argument given by the user in the constructor. The dictionary also contains a key
"label" which is the class label (specified by the user in the constructor) of the image. Note:
for the scene in this example there is no semantic data which is why the semantic output
below is not represented.

.. code:: py

    # Give the extractor a path to the scene
    scene_filepath = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"

    # Instantiate an extractor. The only required argument is the scene filepath
    extractor = ImageExtractor(
        scene_filepath,
        labels=[0.0],
        img_size=(512, 512),
        output=["rgba", "depth", "semantic"],
    )

    # Index in to the extractor like a normal python list
    sample = extractor[0]

    # Or use slicing
    samples = extractor[1:4]
    for sample in samples:
        display_sample(sample)


.. image:: ../images/extractor-example-output.png


`Integrating with Pytorch Datasets`_
====================================

It is very easy to plug an ImageExtractor into a PyTorch Datasets and DataLoaders for end to end
training in PyTorch models without writing to disk. For a great tutorial on how to use PyTorch Dataset
and DataLoader, refer to this guide: https://pytorch.org/tutorials/beginner/data_loading_tutorial.html

.. code:: py

    class HabitatDataset(Dataset):
        def __init__(self, extractor):
            self.extractor = extractor

        def __len__(self):
            return len(self.extractor)

        def __getitem__(self, idx):
            sample = self.extractor[idx]
            output = {
                "rgba": sample["rgba"].astype(np.float32)
                / 255.0,  # dataloader requires certain types
                "label": sample["label"],
            }
            return output


    class TrivialNet(nn.Module):
        def __init__(self):
            super(TrivialNet, self).__init__()

        def forward(self, x):
            x = F.relu(x)
            return x


    dataset = HabitatDataset(extractor)
    dataloader = DataLoader(
        dataset, batch_size=2, num_workers=0
    )  # Sim can only run in 1 process
    net = TrivialNet()

    for i, sample_batch in enumerate(dataloader):
        img, label = sample_batch["rgba"], sample_batch["label"]
        img = img.permute(0, 3, 1, 2)  # Reshape to PyTorch format for convolutions
        out = net(img)


`Appendix`_
===========

I'll explain briefly how the image extraction is actually done so that others can make changes
if necessary. When the user creates a ImageExtractor, the following sequence of events happen:

1. A Simulator class is created and a 2D topdown view of the scene is generated
2. Using the topdown view, the PoseExtractor class creates a grid of points spaced equally across the topdown view
3. For each grid point, the PoseExtractor uses a predefined method for extracting good camera poses to find the
    closest 'point of interest'. For example, one method for pose extraction is to breadth-first-search from every
    gridpoint to find the closest point of interest. A point of interest is a point specified by the class
    labels argument to ImageExtractor.
4. The PoseExtractor returns a list of poses, where each pose contains (position, rotation, label)
    information. When it comes time for the ImageExtractor to return an image to the user, these poses are
    used to set the agent state within the simulator.


Make sure to close the simulator after using it (explained above) if you want to instantiate another one
at a later time!

.. code:: py

    extractor.close()
