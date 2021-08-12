# Datasets commonly used with Habitat-Sim

## Habitat-Matterport 3D Research Dataset (HM3D)

Details: [https://aihabitat.org/datasets/hm3d/](https://aihabitat.org/datasets/hm3d/]).

Getting access: [https://matterport.com/habitat-matterport-3d-research-dataset](https://matterport.com/habitat-matterport-3d-research-dataset)

Github page with download links: [https://github.com/matterport/habitat-matterport-3dresearch](https://github.com/matterport/habitat-matterport-3dresearch)

After getting access to the dataset, you can download manually or programmatically via Habitat's data download utility.

## Downloading with the download utility

First, you will need to generate a matterport API Token:

1. Navigate to https://my.matterport.com/settings/account/devtools
1. Generate an API token
1. Your API token ID then functions as your username, passed to the download script with `--username`,
 and your API token secret functions as your password,
 passed to the download script with `--password`.
 Note: Make sure to write your API token secret down, you can't reveal it again!

Now, you are ready to download. For example, to download the minival split, use:
```
python -m habitat_sim.utils.datasets_download --username <api-token-id> --password <api-token-secret> --uids hm3d_minival
```

By default the download script will only download what is needed for habitat-sim.  You can add `_full` to the uid to download the raw glbs and the obj+mtl's in addition to what is needed for use with habitat-sim.

## Matterport3D (MP3D) dataset

Details: [https://niessner.github.io/Matterport/](https://niessner.github.io/Matterport/).

Github: [https://github.com/niessner/Matterport](https://github.com/niessner/Matterport)

MP3D dataset for use with Habitat can be downloaded using the official [Matterport3D](https://niessner.github.io/Matterport/) download script as follows: `python download_mp.py --task habitat -o path/to/download/`.
Note that this download script requires python 2.7 to run.

You only need the habitat zip archive and not the entire Matterport3D dataset.

## Gibson and 3DSceneGraph datasets

- The Gibson dataset for use with Habitat can be downloaded by agreeing to the terms of use in the [Gibson](https://github.com/StanfordVL/GibsonEnv#database) repository.

- Semantic information for Gibson is available from the [3DSceneGraph](https://3dscenegraph.stanford.edu/) dataset. The semantic data will need to be converted before it can be used within Habitat:
   ```bash
   tools/gen_gibson_semantics.sh /path/to/3DSceneGraph_medium/automated_graph /path/to/GibsonDataset /path/to/output
   ```
   To use semantics, you will need to enable the semantic sensor.

## Replica Dataset

Details and download: [https://github.com/facebookresearch/Replica-Dataset](https://github.com/facebookresearch/Replica-Dataset).

To work with the Replica dataset, you need a file called ```sorted_faces.bin``` for each model. Such files (1 file per model), along with a convenient setup script can be downloaded from here: [sorted_faces.zip](http://dl.fbaipublicfiles.com/habitat/sorted_faces.zip). You need:
```
  - Download the file from the above link;
  - Unzip it;
  - Use the script within to copy each data file to its corresponding folder (You will have to provide the path to the folder containing all replica models. For example, ~/models/replica/);
```
Note: To obtain the best rendering results, use the `<path to replica>/<scene_name>/mesh.ply` to load the PTex mesh.

## ReplicaCAD

Details and download instructions: [https://aihabitat.org/datasets/replica_cad/](https://aihabitat.org/datasets/replica_cad/).

## ScanNet

The official ScanNet data can be downloaded here: [http://www.scan-net.org/](http://www.scan-net.org/). To use ScanNet scans with habitat-sim, the `scene_*.ply` files need to be converted to glTF format (`*.glb`). For example, using [assimp](https://github.com/assimp/assimp):

```
assimp export <PLY FILE> <GLB PATH>
```

The exported `*.glb` files can directly be used with habitat-sim versions >= 2.0.

Note: Depending on the configured radius and height of the agent, certain scans may have no navigable locations on the navmesh (~200). These scenes can be filtered out by checking if `sim.pathfinder.is_loaded` is False.
