Habitat Sim Image Extractor Tutorial
####################################

.. contents::
    :class: m-block m-default


This notebook will go over how to use the image extraction API in Habitat Sim and the different
user options available. It will also over how to create a custom pose extractor, which is how
the image extraction API programmatically defines camera poses for the images.

.. code:: py

    import os

    import matplotlib.pyplot as plt
    import numpy as np
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    from torch.utils.data import DataLoader, Dataset

    from habitat_sim.utils.data import ImageExtractor


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

* labels: List[float]: Numeric labels of the type of images the user wants to extract. Currently we only.
    support extracting images of 'unnavigable points' like walls. In the future we hope to extend this
    functionality to allow the user to specify more unique class labels, but for now this argument is
    not that useful.
* img_size: tuple: The size of images to be output in the format (height, width).
* output: List[str]: A list of the different output image types the user can obtain. The options are
    any of 'rgba', 'depth', and 'semantic'. Default is rgba.
* pose_extractor_name: str: Name of the pose extractor from habitat_sim.registry.
* shuffle: bool: Whether or not to shuffle the extracted images.
* split: tuple: The train/test split. Must add up to 100.
* use_caching: bool: Whether or not to cache (up to capacity) images in memory rather than generating them
    from the simulator on the fly.
* pixels_per_meter: float: Granularity of the topdown view used for setting the camera positions in the pose
    extractor.


`Using the Extractor`_
======================

The extractor can be indexed and sliced like a normal python list. Internally, indexing into
the extractor sets an agent position and rotation within the simulator and returns the corresponding
agent observation. Indexing returns a dictionary that contains an image of each type specified in
the "output" argument given by the user in the constructor. Note: for the scene in this example
there is no semantic data which is why the semantic output below is not represented.

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
        img = img.permute(0, 3, 1, 2)
        out = net(img)


`Writing Custom Pose Extractors`_
=================================

The pose extractor defines how camera poses are programmatically determined so that the image
extractor knows how to manipulated the camera position and angle to extract an image from the
simulator. All custom pose extractors must inherit from the PoseExtractor abstract class and
override the extract_poses method. Further, you must register the pose extractor using
habitat_sim.registry.

The extract_poses method takes three required arguments:

* labels: List[float]: A list of numeric labels of points we are looking for
* view: numpy.ndarray: A 2 dimensional array representing the topdown view of a scene.
* fp: str: The filepath of the scene.

The job of the extract_poses method is to return a list of poses where each pose is a four-tuple
of (<camera position>, <point of interest>, <label>, <scene filepath>). The camera position is the
coordinates of the camera in the space of the topdown view. The point of interest is the position where
the camera will be pointing in the space of the topdown view. The label and filepath are the class
label of what the camera is pointing at and the scene filepath the pose comes from respectively. Below
is an example of a pose extractor that simply chooses some random navigable points and looks forward.

.. code:: py
    import numpy as np
    import habitat_sim.registry as registry

    from habitat_sim.utils.data import ImageExtractor, PoseExtractor

    @registry.register_pose_extractor(name="random_pose_extractor")
    class RandomPoseExtractor(PoseExtractor):
        def __init__(self, topdown_views, pixels_per_meter=0.1):
            super().__init__(topdown_views, pixels_per_meter)

        def extract_poses(self, labels, view, fp):
            height, width = view.shape
            num_random_points = 4
            points = []
            while len(points) < num_random_points:
                row, col = np.random.randint(0, height), np.random.randint(0, width)

                # Convenient method to check if a point is navigable
                if self._valid_point(row, col, view):
                    points.append((row, col))

            poses = []
            for point in points:
                r, c = point
                point_of_interest = (r - 1, c) # Just look forward
                pose = (point, point_of_interest, 0.0, fp) # Different labels not currently supported, so just pass a 0.0 through
                poses.append(pose)

            return poses

    scene_filepath = "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    extractor = ImageExtractor(
        scene_filepath,
        labels=[0.0],
        pose_extractor_name="random_pose_extractor"
    )


.. image:: ../images/random-images.png


In the above code, we registered a new pose extractor with habitat sim and then used the name of
the new pose extractor in the ImageExtractor constructor. One final note is that the init method
should take in two arguments and pass them up to the PoseExtractor constructor using super(). This
is necessary.


`Appendix`_
===========

I'll explain briefly how image extraction is done using the default pose extractor so that others
can make changes if necessary. When the user creates a ImageExtractor, the following sequence of
events happen:

1. A Simulator class is created and a 2D topdown view of the scene is generated
2. Using the topdown view, the PoseExtractor class creates a grid of points spaced equally across the topdown view
3. For each grid point, the PoseExtractor uses a predefined method for extracting good camera poses to find the
    closest 'point of interest'. For example, one method for pose extraction is to breadth-first-search from every
    gridpoint to find the closest point of interest. A point of interest is a point specified by the class
    labels argument to ImageExtractor.
4. The PoseExtractor returns a list of poses, where each pose contains (position, rotation, label, filepath)
    information. When it comes time for the ImageExtractor to return an image to the user, these poses are
    used to set the agent state within the simulator.


Make sure to close the simulator after using it (explained above) if you want to instantiate another one
at a later time!

.. code:: py

    extractor.close
