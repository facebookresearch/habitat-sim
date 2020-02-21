import collections
import copy
import math
import os

import matplotlib.pyplot as plt
import numpy as np

import habitat_sim
import habitat_sim.bindings as hsim
from examples.settings import make_cfg
from habitat_sim.agent import AgentState
from habitat_sim.utils.common import quat_from_two_vectors
from habitat_sim.utils.filesystem import search_dir_tree_for_ext


def make_config_default_settings(scene_filepath, img_size):
    sim_settings = {
        "width": img_size[1],  # Spatial resolution of the observations
        "height": img_size[0],
        "scene": scene_filepath,  # Scene path
        "default_agent": 0,
        "sensor_height": 1.5,  # Height of sensors in meters
        "color_sensor": True,  # RGB sensor
        "semantic_sensor": True,  # Semantic sensor
        "depth_sensor": True,  # Depth sensor
        "silent": True,
    }

    return make_cfg(sim_settings)


class ImageExtractor:
    r"""Main class that extracts data by creating a simulator and generating a topdown map from which to
    iteratively generate image data.

    :property scene_filepath: The location of the .glb file given to the simulator
    :property labels: class labels of things to tather images of
    :property cfg: configuration for simulator of type SimulatorConfiguration
    :property sim: Simulator object
    :property res: Resolution of topdown map. 0.1 means each pixel in the topdown map
        represents 0.1 x 0.1 meters in the coordinate system of the pathfinder
    :property tdv: TopdownView object
    :property topdown_view: The actual 2D array representing the topdown view
    :property pose_extractor: PoseExtractor object
    :property poses: list of camera poses gathered from pose_extractor
    :property label_map: maps lable numbers on the topdown map to their name
    :property out_name_to_sensor_name: maps name of output to the sensor same corresponding to that output
    :property output: list of output names that the user wants e.g. ['rgb', 'depth']
    """

    def __init__(
        self,
        filepath,
        labels=[0.0],
        img_size=(512, 512),
        output=["rgb"],
        sim=None,
        shuffle=True,
        split=(70, 30),
    ):
        if sum(split) != 100:
            raise Exception("Train/test split must sum to 100.")

        self.scene_filepaths = None
        self.cur_fp = None
        if os.path.isdir(filepath):
            self.scene_filepaths = search_dir_tree_for_ext(filepath, ".glb")
        else:
            self.scene_filepaths = [filepath]
            self.cur_fp = filepath

        self.labels = set(labels)
        self.img_size = img_size
        self.cfg = make_config_default_settings(self.scene_filepaths[0], self.img_size)

        if sim is None:
            sim = habitat_sim.Simulator(self.cfg)
        else:
            # If a sim is provided we have to make a new cfg
            self.cfg = config_sim(sim.config.sim_cfg.scene.id, img_size)
            sim.reconfigure(self.cfg)

        self.sim = sim
        self.res = 0.1
        self.tdv_fp_ref_triples = self.precomute_tdv_and_refs(
            self.sim, self.scene_filepaths, self.res
        )

        # self.tdv = TopdownView(self.sim, self.res)
        # self.topdown_view = self.tdv.topdown_view
        self.pose_extractor = PoseExtractor(
            self.tdv_fp_ref_triples, self.sim, self.res
        )
        self.poses = self.pose_extractor.extract_poses(
            labels=self.labels
        )  # list of poses

        if shuffle:
            np.random.shuffle(self.poses)

        self.train, self.test = self._handle_split(split, self.poses)
        self.mode = "full"
        self.mode_to_data = {
            "full": self.poses,
            "train": self.train,
            "test": self.test,
            None: self.poses,
        }

        self.instance_id_to_name = self._generate_label_map(self.sim.semantic_scene)
        self.out_name_to_sensor_name = {
            "rgb": "color_sensor",
            "depth": "depth_sensor",
            "semantic": "semantic_sensor",
        }
        self.output = output

    def __len__(self):
        return len(self.mode_to_data[self.mode])

    def __getitem__(self, idx):
        if isinstance(idx, slice):
            start, stop, step = idx.start, idx.stop, idx.step
            if start is None:
                start = 0
            if stop is None:
                stop = len(self.mode_to_data[self.mode])
            if step is None:
                step = 1

            return [
                self.__getitem__(i)
                for i in range(start, stop, step)
                if i < len(self.mode_to_data[self.mode])
            ]

        mymode = self.mode.lower()
        poses = self.mode_to_data[mymode]
        pos, rot, label, fp = poses[idx]

        # Only switch scene if it is different from the last one accessed
        if fp != self.cur_fp:
            self.sim.reconfigure(make_config_default_settings(fp, self.img_size))
            self.cur_fp = fp

        new_state = AgentState()
        new_state.position = pos
        new_state.rotation = rot
        self.sim.agents[0].set_state(new_state)
        obs = self.sim.get_sensor_observations()
        sample = {
            out_name: obs[self.out_name_to_sensor_name[out_name]]
            for out_name in self.output
        }
        #sample["label"] = self.label_map[label]

        return sample

    def precomute_tdv_and_refs(self, sim, scene_filepaths, res):
        tdv_fp_ref = []
        for filepath in scene_filepaths:
            cfg = make_config_default_settings(filepath, self.img_size)
            sim.reconfigure(cfg)
            ref_point = self._get_pathfinder_reference_point(sim.pathfinder)
            tdv = TopdownView(sim, ref_point[1], res=res)
            tdv_fp_ref.append((tdv, filepath, ref_point))
        
        return tdv_fp_ref

    def close(self):
        self.sim.close()
        del self.sim

    def set_mode(self, mode):
        mymode = mode.lower()
        if mymode not in ["full", "train", "test"]:
            raise Exception(
                f'Mode {mode} is not a valid mode for ImageExtractor. Please enter "full, train, or test"'
            )

        self.mode = mymode

    def get_semantic_class_names(self):
        class_names = list(set(
            name for name in self.instance_id_to_name.values() if name != 'background'
        ))
        class_names = ['background'] + class_names # Make sure background is index 0
        return class_names

    def _handle_split(self, split, poses):
        train, test = split
        num_poses = len(self.poses)
        last_train_idx = int((train / 100) * num_poses)
        train_poses = poses[:last_train_idx]
        test_poses = poses[last_train_idx:]
        return train_poses, test_poses

    def _get_pathfinder_reference_point(self, pf):
        bound1, bound2 = pf.get_bounds()
        startw = min(bound1[0], bound2[0])
        starth = min(bound1[2], bound2[2])
        starty = pf.get_random_navigable_point()[
            1
        ]  # Can't think of a better way to get a valid y-axis value
        return (startw, starty, starth)  # width, y, height

    def _generate_label_map(self, scene, verbose=False):
        if verbose:
            print(f"House has {len(scene.levels)} levels, {len(scene.regions)} regions and {len(scene.objects)} objects")
            print(f"House center:{scene.aabb.center} dims:{scene.aabb.sizes}")

        instance_id_to_name = {}
        for obj in scene.objects:
            if obj and obj.category:
                obj_id = int(obj.id.split('_')[-1])
                instance_id_to_name[obj_id] = obj.category.name()
    
        return instance_id_to_name


class TopdownView(object):
    def __init__(self, sim, y_axis_val, res=0.1):
        self.topdown_view = np.array(sim.pathfinder.get_topdown_view(res, y_axis_val)).astype(
            np.float64
        )


class PoseExtractor(object):
    r"""Class that takes in a topdown view and pathfinder and determines a list of reasonable camera poses

    :property topdown_view: 2D array representing topdown view of scene
    :property pathfinder: the pathfinder from the Simulator object
    :property res: resolution of the topdown view (explained in HabitatDataset)
    :property gridpoints: list of positions for the camera
    :property dist: distance between each camera position
    """

    def __init__(self, topdown_views, sim, res=0.1):
        self.tdv_fp_ref_triples = topdown_views
        self.sim = sim
        self.res = res

    def extract_poses(self, labels):
        poses = []
        for tdv, fp, ref_point in self.tdv_fp_ref_triples:
            view = tdv.topdown_view
            # Determine the physical spacing between each camera position
            x, z = view.shape
            dist = min(x, z) // 10
            poses.extend(self.extract_poses_single_scene(labels, view, fp, dist, ref_point))

        return np.array(poses)

    def extract_poses_single_scene(self, labels, view, fp, dist, ref_point):
        height, width = view.shape
        n_gridpoints_width, n_gridpoints_height = (
            width // dist - 1,
            height // dist - 1,
        )
        gridpoints = []
        for h in range(n_gridpoints_height):
            for w in range(n_gridpoints_width):
                point = (dist + h * dist, dist + w * dist)
                if self.valid_point(*point, view):
                    gridpoints.append(point)

        # Find the closest point of the target class to each gridpoint
        poses = []
        cpis = []
        for point in gridpoints:
            closest_point_of_interest, label = self._bfs(point, labels, view, dist)
            if closest_point_of_interest is None:
                continue

            poses.append((point, closest_point_of_interest, label, fp))
            cpis.append(closest_point_of_interest)

        # Convert from topdown map coordinate system to that of the pathfinder
        startw, starty, starth = ref_point
        for i, pose in enumerate(poses):
            pos, cpi, label, filepath = pose
            r1, c1 = pos
            r2, c2 = cpi
            new_pos = np.array([startw + c1 * self.res, starty, starth + r1 * self.res])
            new_cpi = np.array([startw + c2 * self.res, starty, starth + r2 * self.res])
            cam_normal = new_cpi - new_pos
            new_rot = self._compute_quat(cam_normal)
            poses[i] = (new_pos, new_rot, label, filepath)

        return poses

    def valid_point(self, row, col, view):
        return view[row][col] == 1.0

    def is_point_of_interest(self, point, labels, view):
        r, c = point
        is_interesting = False
        if view[r][c] in labels:
            is_interesting = True

        return is_interesting, view[r][c]

    def _show_topdown_view(self, cmap="seismic_r"):
        fig=plt.figure(figsize=(12.0, 12.0))
        columns = 4
        rows = math.ceil(len(self.tdv_fp_ref_triples) / columns)
        for i in range(1, columns * rows + 1):
            if i > len(self.tdv_fp_ref_triples):
                break

            img = self.tdv_fp_ref_triples[i - 1][0].topdown_view
            fig.add_subplot(rows, columns, i)
            plt.xticks([])
            plt.yticks([])
            plt.imshow(img, cmap=cmap)

        plt.show()

    def _compute_quat(self, cam_normal):
        """Rotations start from -z axis"""
        return quat_from_two_vectors(np.array([0, 0, -1]), cam_normal)

    def _bfs(self, point, labels, view, dist):
        step = 3  # making this larger really speeds up BFS

        def get_neighbors(p):
            r, c = p
            return [
                (r - step, c - step),
                (r - step, c),
                (r - step, c + step),
                (r, c - step),
                (r, c + step),
                (r + step, c - step),
                (r + step, c),
                (r + step, c + step),
            ]

        point_row, point_col = point
        bounding_box = [
            point_row - 2 * dist,
            point_row + 2 * dist,
            point_col - 2 * dist,
            point_col + 2 * dist,
        ]
        in_bounds = (
            lambda row, col: bounding_box[0] <= row <= bounding_box[1]
            and bounding_box[2] <= col <= bounding_box[3]
        )
        is_valid = lambda row, col: 0 <= row < len(view) and 0 <= col < len(view[0])
        visited = (
            set()
        )  # Can use the topdown view as visited set to save space at cost of time to reset it for each bfs
        q = collections.deque([(point, 0)])
        while q:
            cur, layer = q.popleft()
            if not in_bounds(*cur):  # No point of interest found within bounding box
                return None, None

            visited.add(cur)
            is_point_of_interest, label = self.is_point_of_interest(cur, labels, view)
            if is_point_of_interest:
                if layer > dist / 2:
                    return cur, label
                else:
                    return None, None

            for n in get_neighbors(cur):
                if n not in visited and is_valid(*n):
                    q.append((n, layer + step))

        return None, None
