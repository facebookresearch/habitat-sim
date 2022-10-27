# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import time
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Union

import magnum as mn
import numpy as np
from fairmotion.core import motion
from fairmotion.data import amass
from fairmotion.ops import motion as motion_ops

from habitat_sim.logging import LoggingContext, logger

LoggingContext.reinitialize_from_env()
logger.setLevel("INFO")

#### Constants ###
ROOT, FIRST, LAST = 0, 0, -1


class MType(Enum):
    """
    This emun class represents the two ways that motion data can be setup and used.
    Transitive motion data is used to configure and make available cyclical motion data
    that is needed for the character to traverse paths and displace the character between
    Scenic actions. Scenic motion data is more performative in nature and is used to
    configure motion capture data that usually plays out in a specific location.
    """

    TRANSITIVE = 0
    SCENIC = 1


class Motions:
    """
    The Motions class is collection of stats that will hold the different movement motions
    for the character to use when following a path. The character is left-footed so that
    is our reference for with step the motions assume first.
    """

    @dataclass
    class MotionData:
        """
        A class intended to handle precomputations of utilities of the motion we want to
        load into the character.
        """

        # transitive motion is necessary for scenic motion build
        def __init__(
            self,
            motion_: motion.Motion,
            type_: MType,
            transitive_motion_=None,
            scenic_direction_offset: int = 0,
        ) -> None:
            logger.info("Loading Motion data...")
            if type_ == MType.TRANSITIVE:
                # primary
                self.motion = motion_
                self.type = MType.TRANSITIVE
                self.poses = motion_.poses
                self.fps = motion_.fps
                self.num_of_frames: int = len(motion_.poses)
                self.map_of_total_displacement: float = []
                self.center_of_root_drift: mn.Vector3 = mn.Vector3()
                self.time_length = self.num_of_frames * (1.0 / motion_.fps)

                # intermediates
                self.translation_drifts: List[mn.Vector3] = []
                self.forward_displacements: List[mn.Vector3] = []
                self.root_orientations: List[mn.Quaternion] = []

                # first and last frame root position vectors
                f = motion_.poses[0].get_transform(ROOT, local=False)[0:3, 3]
                f = mn.Vector3(f)
                l = motion_.poses[LAST].get_transform(ROOT, local=False)[0:3, 3]
                l = mn.Vector3(l)

                # axis that motion uses for up and forward
                self.direction_up = mn.Vector3.z_axis()
                forward_V = (l - f) * (mn.Vector3(1.0, 1.0, 1.0) - mn.Vector3.z_axis())
                self.direction_forward = forward_V.normalized()

                ### Fill derived data structures ###
                # fill translation_drifts and forward_displacements
                for i in range(self.num_of_frames):
                    j = i + 1
                    if j == self.num_of_frames:
                        # interpolate forward and drift from nth vectors and 1st vectors and push front
                        self.forward_displacements.insert(
                            0,
                            (
                                (
                                    self.forward_displacements[LAST]
                                    + self.forward_displacements[0]
                                )
                                * 0.5
                            ),
                        )
                        self.translation_drifts.insert(
                            0,
                            (
                                (
                                    self.translation_drifts[LAST]
                                    + self.translation_drifts[0]
                                )
                                * 0.5
                            ),
                        )
                        break

                    # root translation
                    curr_root_t = motion_.poses[i].get_transform(ROOT, local=False)[
                        0:3, 3
                    ]
                    next_root_t = motion_.poses[j].get_transform(ROOT, local=False)[
                        0:3, 3
                    ]

                    delta_P_vector = mn.Vector3(next_root_t - curr_root_t)
                    forward_vector = delta_P_vector.projected(self.direction_forward)
                    drift_vector = delta_P_vector - forward_vector

                    self.forward_displacements.append(forward_vector)
                    self.translation_drifts.append(drift_vector)

                j, summ = 0, 0
                # fill translation_drifts and forward_displacements
                for i in range(self.num_of_frames):
                    curr_root_t = motion_.poses[i].get_transform(ROOT, local=False)[
                        0:3, 3
                    ]
                    prev_root_t = motion_.poses[j].get_transform(ROOT, local=False)[
                        0:3, 3
                    ]

                    # fill map_of_total_displacement
                    summ += (
                        mn.Vector3(curr_root_t - prev_root_t)
                        .projected(self.direction_forward)
                        .length()
                    )
                    self.map_of_total_displacement.append(summ)
                    j = i

                # fill root_orientations
                for pose in motion_.poses:
                    root_T = pose.get_transform(ROOT, local=False)
                    root_rotation = mn.Quaternion.from_matrix(
                        mn.Matrix3x3(root_T[0:3, 0:3])
                    )
                    self.root_orientations.append(root_rotation)

                # get center of drift
                summ = mn.Vector3()
                for pose in motion_.poses:
                    root_T = mn.Matrix4(pose.get_transform(ROOT, local=False))
                    root_T.translation *= (
                        mn.Vector3(1.0, 1.0, 1.0) - self.direction_forward
                    )
                    summ += root_T.translation
                self.center_of_root_drift = summ / self.num_of_frames

            if type_ == MType.SCENIC:
                # must pass in a transitive motion
                assert transitive_motion_ is not None

                # primary
                self.motion = motion_
                self.type = MType.SCENIC
                self.poses = motion_.poses
                self.fps = motion_.fps
                self.num_of_frames: int = len(motion_.poses)
                self.time_length = self.num_of_frames * (1.0 / motion_.fps)
                self.transitive_motion_: Motions.MotionData = transitive_motion_

                # axis that motion uses for up and forward
                self.direction_up = mn.Vector3.z_axis()
                self.direction_forward = mn.Vector3.x_axis()

                # add scenic_direction offset
                angle = scenic_direction_offset
                rotate = mn.Quaternion.rotation(mn.Deg(angle), self.direction_up)
                self.direction_forward = rotate.transform_vector(self.direction_forward)

                # edge cases
                self.start_translation = motion_.poses[FIRST].get_transform(
                    ROOT, local=False
                )[0:3, 3]
                self.final_translation = motion_.poses[LAST].get_transform(
                    ROOT, local=False
                )[0:3, 3]

    logger.info("Loading Motion data...")
    motion_files = [
        "data/fairmotion/amass_test_data/CMU/CMU/10/10_04_poses.npz",  # [0] cycle walk
        "data/fairmotion/amass_test_data/CMU/CMU/09/09_01_poses.npz",  # [1] cycle run
        "data/fairmotion/amass_test_data/CMU/CMU/13/13_08_poses.npz",  # [2] drink beverage
        "data/fairmotion/amass_test_data/CMU/CMU/13/13_22_poses.npz",  # [3] wash window
        "data/fairmotion/amass_test_data/CMU/CMU/13/13_23_poses.npz",  # [4] sweep floor
        "data/fairmotion/amass_test_data/CMU/CMU/13/13_29_poses.npz",  # [5] jumping jacks
        "data/fairmotion/amass_test_data/CMU/CMU/13/13_10_poses.npz",  # [6] reach for
    ]
    bm_path = "data/fairmotion/amass_test_data/smplh/male/model.npz"
    motion_data = amass.load_parallel(motion_files, bm_path=bm_path)

    ### TRANSITIVE ###
    # all motions must have same fps for this implementation, so use first motion to set global
    fps = motion_data[0].fps

    # Standing pose that must be converted (amass -> habitat joint positions)
    standing_pose = motion_data[0].poses[0]

    # Walk-to-walk cycle
    walk_to_walk = MotionData(
        motion_ops.cut(motion_data[0], 300, 430), MType.TRANSITIVE
    )

    # Run-to-run cycle
    run_to_run = MotionData(motion_ops.cut(motion_data[1], 3, 89), MType.TRANSITIVE)

    ### SCENIC ###
    drink_beverage = MotionData(
        motion_data[2], MType.SCENIC, transitive_motion_=walk_to_walk
    )
    wash_window = MotionData(
        motion_ops.cut(motion_data[3], 3040, 4090),
        MType.SCENIC,
        transitive_motion_=walk_to_walk,
    )
    sweep_floor = MotionData(
        motion_data[4], MType.SCENIC, transitive_motion_=walk_to_walk
    )
    jumping_jacks = MotionData(
        motion_data[5],
        MType.SCENIC,
        transitive_motion_=run_to_run,
        scenic_direction_offset=90,
    )
    reach_for = MotionData(
        motion_data[6],
        MType.SCENIC,
        transitive_motion_=walk_to_walk,
        scenic_direction_offset=-80,
    )


@dataclass
class PathData:
    """
    The PathData class is purposed to instantiate with given path points, and used
    to manage the path data fro the current path following sequence.
    """

    def __init__(self, path_points) -> None:
        self.points = path_points
        self.length = self.calc_path_length(path_points)
        self.time = 0.0

    # [REFACTOR] I would like to change this to a static method, once I figure out how
    def calc_path_length(self, path_points: List[mn.Vector3]) -> float:
        # get path length
        i, j, summ = 0, 0, 0.0
        while i < len(path_points):
            summ += (mn.Vector3(path_points[i] - path_points[j])).length()
            j = i
            i += 1
        return summ

    def __str__(self) -> str:
        return f"PathData(points={type(self.points)}, length={self.length}, time={self.time})"


@dataclass
class ActionOrder:
    """
    The ActionOrder class holds the data necessary to command the pathfollower character
    to perform a scenic motion at a specified location.
    """

    def __init__(
        self,
        motion_data: Motions.MotionData,
        location: Optional[Union[mn.Vector3, np.array]] = None,
        facing: Optional[Union[mn.Vector3, np.array]] = None,
    ) -> None:
        self.motion_data = motion_data

        self.location = None
        if location is not None:
            self.location = mn.Vector3(location)

        self.facing = None
        if facing is not None:
            self.facing = mn.Vector3(facing)

        # Transitive motions must have a location
        if motion_data.type == MType.TRANSITIVE:
            assert location is not None


# keeps track of the motion key frame preview modes
class Preview(Enum):
    OFF = 0
    KEYFRAMES = 1
    TRAJECTORY = 2
    ALL = 3


# keeps track of the activity that intances model is  participating in currently
class Activity(Enum):
    NONE = 0
    MOTION_STAGE = 1
    PATH_FOLLOW_SEQ = 2
    SEQUENCE = 3


class Timer:
    """
    Timer class used to keep track of time.
    """

    start_time = 0.0

    @staticmethod
    def start() -> None:
        """
        Starts timer and resets previous frame time to the start time
        """
        Timer.start_time = time.time()

    @staticmethod
    def check() -> float:
        """
        Returns time since last call to `start()`. Only accurate if `start()`
        has been called previously.
        """
        return time.time() - Timer.start_time
