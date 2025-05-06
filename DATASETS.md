# HowTo Use Common Supported Datasets with Habitat-Sim

🤗 View the open-source collection of Habitat-ready datasets and test assets on Hugging Face at https://huggingface.co/ai-habitat!

## Table of contents
   1. [Habitat test scenes](#habitat-test-scenes)
   1. [Habitat-Matterport 3D Research Dataset (HM3D)](#habitat-matterport-3d-research-dataset-hm3d)
   1. [Habitat Synthetic Scenes Dataset (HSSD)](#habitat-synthetic-scene-dataset-hssd)
   1. [AI2-THOR (Habitat)](#ai2-thor-habitat)
   1. [Matterport3D (MP3D) dataset](#matterport3d-mp3d-dataset)
   1. [Gibson and 3DSceneGraph datasets](#gibson-and-3dscenegraph-datasets)
   1. [Replica Dataset](#replica-dataset)
   1. [ReplicaCAD](#replicacad)
   1. [ScanNet](#scannet)
   1. [YCB Benchmarks - Object and Model Set](#ycb-benchmarks---object-and-model-set)
   1. [Previewing dataset assets using Habitat-Sim's viewers](#previewing-dataset-assets-using-habitat-sims-viewers)

___

## Habitat test scenes
[🤗 Browse on Hugging Face 🤗](https://huggingface.co/datasets/ai-habitat/habitat_test_scenes)

We provide 3 example scenes for performing unit tests in habitat-sim. These can be programmatically downloaded via Habitat's data download utility.

```
python -m habitat_sim.utils.datasets_download --uids habitat_test_scenes --data-path data/
```

We also provide PointNav episodes sampled from these scenes for performing unit tests in habitat-lab. These can also be downloaded using Habitat's data download utility.

```
python -m habitat_sim.utils.datasets_download --uids habitat_test_pointnav_dataset --data-path data/
```

___

## Habitat-Matterport 3D Research Dataset (HM3D)

Details: [https://aihabitat.org/datasets/hm3d/](https://aihabitat.org/datasets/hm3d/).

Getting access: [https://matterport.com/habitat-matterport-3d-research-dataset](https://matterport.com/habitat-matterport-3d-research-dataset)

Github page with download links: [https://github.com/matterport/habitat-matterport-3dresearch](https://github.com/matterport/habitat-matterport-3dresearch)

After getting access to the dataset, you can download manually or programmatically via Habitat's data download utility.

### Downloading HM3D with the download utility

First, you will need to generate a matterport API Token:

1. Navigate to https://my.matterport.com/settings/account/devtools
1. Generate an API token
1. Your API token ID then functions as your username, passed to the download script with `--username`,
 and your API token secret functions as your password,
 passed to the download script with `--password`.
 Note: Make sure to write your API token secret down, you can't reveal it again!

Now, you are ready to download. For example, to download the minival split, use:
```
python -m habitat_sim.utils.datasets_download --username <api-token-id> --password <api-token-secret> --uids hm3d_minival_v0.2
```

By default, downloading the data for train/val/example scenes also pulls in the semantic annotations and configs for [HM3D-Semantics v0.2](https://aihabitat.org/datasets/hm3d-semantics/). To download only the semantic files for these splits, use the uid `hm3d_semantics`.


By default the download script will only download what is needed for Habitat-Sim. You can add `_full` to the uid to download the raw glbs in addition to what is needed for use with Habitat-Sim.

### Loading semantics for HM3D

First, ensure that the corresponding file structure exists (see minival example below):

```
> ls <PATH TO HM3D>
hm3d_annotated_basis.scene_dataset_config.json    train/
hm3d_basis.scene_dataset_config.json              val/
minival/

> ls <PATH TO HM3D>/minival
00800-TEEsavR23oF/    00806-tQ5s4ShP627/
00801-HaxA7YrQdEC/    00807-rsggHU7g7dh/
00802-wcojb4TFT35/    00808-y9hTuugGdiq/
00803-k1cupFYWXJ6/    00809-Qpor2mEya8F/
00804-BHXhpBwSMLh/    hm3d_annotated_minival_basis.scene_dataset_config.json
00805-SUHsP6z2gcJ/    hm3d_minival_basis.scene_dataset_config.json

> ls <PATH TO HM3D>/minival/00800-TEEsavR23oF/
TEEsavR23oF.basis.glb       TEEsavR23oF.basis.navmesh
TEEsavR23oF.semantic.glb    TEEsavR23oF.semantic.txt
```
Note that there may be more files in `<PATH TO HM3D>/minival/00800-TEEsavR23oF/` if the full HM3D dataset is downloaded. Most importantly, ensure that the `hm3d_annotated_*`, `*.semantic.glb`, and `*.semantic.txt` files are present.

To load semantic annotations in Habitat-Sim:
* Enable the semantic sensor
* Set the `scene_dataset_config_file` configuration variable

A simple example below:
```
import habitat_sim

backend_cfg = habitat_sim.SimulatorConfiguration()
backend_cfg.scene_id = "<PATH TO HM3D>/minival/00800-TEEsavR23oF/TEEsavR23oF.basis.glb"
backend_cfg.scene_dataset_config_file = "<PATH TO HM3D>/hm3d_annotated_basis.scene_dataset_config.json"

sem_cfg = habitat_sim.CameraSensorSpec()
sem_cfg.uuid = "semantic"
sem_cfg.sensor_type = habitat_sim.SensorType.SEMANTIC

agent_cfg = habitat_sim.agent.AgentConfiguration()
agent_cfg.sensor_specifications = [sem_cfg]

sim_cfg = habitat_sim.Configuration(backend_cfg, [agent_cfg])
sim = habitat_sim.Simulator(sim_cfg)
```

To view the semantic annotations in the C++ viewer, install the latest `main` branch of Habitat-Sim and run the following command:
```
# ./build/viewer if compiled locally
habitat-viewer --dataset '<PATH TO HM3D>/hm3d_annotated_basis.scene_dataset_config.json' TEEsavR23oF
```


To load semantic annotations in habitat-lab:

* Update your config by adding the semantic sensor to the list of agent sensors:

    ```yaml
    # @package _global_

    defaults:
      - /habitat: habitat_config_base
      - /habitat/simulator/agents@habitat.simulator.agents.<your_agent_name>: agent_base
      # add semantic sensor:
      - /habitat/simulator/sim_sensors@habitat.simulator.agents.<your_agent_name>.sim_sensors.semantic_sensor: semantic_sensor
      - /habitat/task/rearrange: <your_task>
      - /habitat/dataset/rearrangement: <your_dataset>
      - _self_

    habitat:
      simulator:
        agents:
          <your_agent_name>:
            sim_sensors:
              # configure semantic sensor:
              semantic_sensor:
                height: 256
                width: 256
    ```
* Set the `habitat.simulator.scene_dataset` configuration value:

    ```yaml
    habitat:
      simulator:
        scene_dataset: "<PATH TO HM3D>/hm3d_annotated_basis.scene_dataset_config.json"
    ```

Note that if you are using the RL environment from habitat-lab, `habitat.simulator.scene_dataset` is overridden by the episode dataset (see [here](https://github.com/facebookresearch/habitat-lab/blob/b1f2d4791a0065d0791001b72a6c96748a5f9ae0/habitat-lab/habitat/core/env.py#L104-L108)). Each episode in the episode dataset must point to the annotation config file (as done in the HM3D ObjectNav dataset [here](https://github.com/facebookresearch/habitat-lab/blob/b1f2d4791a0065d0791001b72a6c96748a5f9ae0/DATASETS.md)).

___

## Habitat Synthetic Scene Dataset (HSSD)

Details: [https://3dlg-hcvc.github.io/hssd/](https://3dlg-hcvc.github.io/hssd/).

You can browse and download the HSSD dataset from instructions provided in the above website.

For quick setup and use with habitat-sim and habitat-lab, register for access [on the huggingface hssd-hab repo page](https://huggingface.co/datasets/hssd/hssd-hab) and then use the `datasets_download.py` script as follows:

```
python -m habitat_sim.utils.datasets_download --username <huggingface-username> --password <huggingface-password> --uids hssd-hab --data-path data/
```

___

## AI2-THOR (Habitat)

Details: [https://huggingface.co/datasets/hssd/ai2thor-hab](https://huggingface.co/datasets/hssd/ai2thor-hab).

You can download Habitat-compatible versions of the iTHOR, RoboTHOR, and ProcTHOR scene datasets from instructions provided in the above website.

___

## Matterport3D (MP3D) dataset


Details: [https://niessner.github.io/Matterport/](https://niessner.github.io/Matterport/).
Github: [https://github.com/niessner/Matterport](https://github.com/niessner/Matterport)

We provide 1 example scene from MP3D for performing unit tests in habitat-sim. This can be programmatically downloaded via Habitat's data download utility.

```
python -m habitat_sim.utils.datasets_download --uids mp3d_example_scene --data-path data/
```


The full MP3D dataset for use with Habitat can be downloaded using the official [Matterport3D](https://niessner.github.io/Matterport/) download script as follows: `python download_mp.py --task habitat -o path/to/download/`. Note that this download script requires python 2.7 to run.

You only need the habitat zip archive and not the entire Matterport3D dataset.

Once you have the habitat zip archive, you should download [this SceneDatasetConfig file](http://dl.fbaipublicfiles.com/habitat/mp3d/config_v1/mp3d.scene_dataset_config.json) and place it in the root directory for the Matterport3D dataset (e.g. Habitat-Sim/data/scene_datasets/mp3d/). This file should then be specified as [the scene dataset config in the SimulatorConfiguration structure](/examples/tutorials/nb_python/ReplicaCAD_quickstart.py#L145) like this example for the ReplicaCAD dataset.

___

## Gibson and 3DSceneGraph datasets

The Gibson dataset for use with Habitat can be downloaded by agreeing to the terms of use in the [Gibson](https://github.com/StanfordVL/GibsonEnv#database) repository.

Semantic information for Gibson is available from the [3DSceneGraph](https://3dscenegraph.stanford.edu/) dataset. The semantic data will need to be converted before it can be used within Habitat:
   ```bash
   tools/gen_gibson_semantics.sh /path/to/3DSceneGraph_medium/automated_graph /path/to/GibsonDataset /path/to/output
   ```
   To use semantics, you will need to enable the semantic sensor.

   Once you have downloaded the Gibson dataset and converted the semantic data, you should download [this SceneDatasetConfig file](http://dl.fbaipublicfiles.com/habitat/gibson/config_v1/gibson_semantic.scene_dataset_config.json) and place it in the root directory for the Gibson dataset (e.g. Habitat-Sim/data/scene_datasets/gibson/). This file should then be specified as [the scene dataset config in the SimulatorConfiguration structure](/examples/tutorials/nb_python/ReplicaCAD_quickstart.py#L145) like this example for the ReplicaCAD dataset.

___

## Replica Dataset

Details and download instructions: [https://github.com/facebookresearch/Replica-Dataset](https://github.com/facebookresearch/Replica-Dataset).

___

## ReplicaCAD
🤗 Browse on Hugging Face ([interactive](https://huggingface.co/datasets/ai-habitat/ReplicaCAD_dataset), [baked_lighting](https://huggingface.co/datasets/ai-habitat/ReplicaCAD_baked_lighting)) 🤗

Details and instructions: [https://aihabitat.org/datasets/replica_cad/](https://aihabitat.org/datasets/replica_cad/).

___

## ScanNet

The official ScanNet data can be downloaded here: [http://www.scan-net.org/](http://www.scan-net.org/). To use ScanNet scans with Habitat-Sim, the `scene_*.ply` files need to be converted to glTF format (`*.glb`). For example, using [assimp](https://github.com/assimp/assimp):

```
assimp export <PLY FILE> <GLB PATH>
```

The exported `*.glb` files can directly be used with Habitat-Sim versions >= 2.0.

Note: Depending on the configured radius and height of the agent, certain scans may have no navigable locations on the navmesh (~200). These scenes can be filtered out by checking if `sim.pathfinder.is_loaded` is False.

___

## YCB Benchmarks - Object and Model Set
[🤗 Browse Habitat-ready asset dataset on Hugging Face 🤗](https://huggingface.co/datasets/ai-habitat/ycb)

Details: [https://www.ycbbenchmarks.com/](https://www.ycbbenchmarks.com/).

> YCB Object and Model Set is designed for facilitating benchmarking in robotic manipulation... The set is associated with a [model database](http://www.ycbbenchmarks.com/object-models/) which provides mesh models and high-resolution RGB-D scans of the objects for easy incorporation into manipulation and planning software platforms.

Quick-start with the dataset_downloader utility:

```
# with conda install
python -m habitat_sim.utils.datasets_download --uids ycb --data-path /path/to/data/

# with source
python /path/to/habitat_sim/src_python/habitat_sim/utils/datasets_download.py --uids ycb --data-path /path/to/data/
```

Load the assets in python by setting [MetadataMediator.active_dataset](https://aihabitat.org/docs/habitat-sim/habitat_sim.metadata.MetadataMediator.html#active_dataset):
```
#load the full YCB dataset into the MetadataMediator
sim.metadata_mediator.active_dataset = "/path/to/data/objects/ycb/ycb.scene_dataset_config.json"

#then instance objects from the ObjectAttributesManager:
chef_can_key = sim.get_object_template_manager().get_file_template_handles("002_master_chef_can")[0]
chef_can_object = sim.get_rigid_object_manager().add_object_by_template_handle(chef_can_key)
```
For more information on using objects in habitat-sim, see the "Habitat-Sim for Interaction" and "Habitat-Sim Advanced Topics" sections of our [ECCV tutorial series](https://aihabitat.org/tutorial/2020/).

To quickly test in the viewer application:
```
#from the habitat-sim directory
# C++
# ./build/viewer if compiling locally
habitat-viewer --use-default-lighting --enable-physics --object-dir ""  --dataset data/objects/ycb/ycb.scene_dataset_config.json -- data/test_assets/scenes/simple_room.glb
```
Then press `'o'` key to add random objects from the dataset.

___

# Previewing dataset assets using  Habitat-Sim's viewers

For datasets with scene dataset configuration support (such as HM3D, ReplicaCAD, MP3D, Gibson, etc) you can preview the assets using one of Habitat's command-line driven viewers, either in c++ or python. When launching the viewer, you should specify not only the desired scene to load, but also the specifying the scene dataset configuration file, to guarantee the assets load and display correctly.  This has the added benefit of providing quick access to other scenes in the same dataset, without requiring a reload of the entire simulation environment from the command line.

If you are using the python [viewer](/examples/viewer.py), the command line parameters to load a scene dataset configuration and a scene file would be (run from the Habitat-Sim source directory):

```
python examples/viewer.py --dataset '<path to desired dataset config>/<desired dataset>.scene_dataset_config.json' --scene '<scene to show>'
```

If you are using the c++ [viewer](/src/utils/viewer/viewer.cpp), the command line parameters to load a scene dataset configuration and a scene file would be (run from the Habitat-Sim source directory):

```
# ./build/viewer if compiled locally
habitat-viewer --dataset '<path to desired dataset config>/<desired dataset>.scene_dataset_config.json' '<scene to show>'
```

To preview other scenes in the same scene dataset in either viewer, use `TAB`/`SHIFT-TAB` to cycle forward/backward through the list of scenes referenced by the loaded scene dataset config.
