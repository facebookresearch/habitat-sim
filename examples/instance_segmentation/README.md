### Semantic and Instance Segmentation

This is a pipeline for training semantic and instance segmentation models on images extracted from scenes within Habitat Sim. The code is modular so that you can plug in different models and parameters.

### Requirements

The code in this example relies on some files from PyTorch's vision repository and are not included in Habitat Sim. These mainly consist of code for evaluation and logging. Run

```
cd examples/instance_segmentation
./download-vision-files.sh
cd ../../
```

to download the files. The package requirements and supported versions are below. This has been tested on Ubuntu 18.04.

```
conda install numpy=1.17
conda install pytorch torchvision cudatoolkit=10.1 -c pytorch
pip install pycocotools
```

### Usage

To train a Mask R-CNN resnet50 backbone model on a specified scene, run

```
python examples/instance_segmentation/main.py --scene /path/to/scene/file
```

Scene is the only required argument. To load in pretrained weights to the model, use the ```--load-path``` flag. For more options, please refer to main.py.
