# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import time
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

import magnum as mn
from fairmotion.data import amass
from fairmotion.ops import motion as motion_ops

#### Constants ###
ROOT, LAST = 0, -1


class MType(Enum):
    TRANSITIVE = 0
    SCENIC = 1


class Motions:
    """
    The Move class is collection of stats that will hold the different movement motions
    for the character to use when following a path. The character is left-footed so that
    is our reference for with step the motions assume first.
    """

    @dataclass
    class MotionData:
        """
        A class intended to handle precomputations of utilities of the motion we want to
        load into the character.
        """

        def __init__(self, motion_, type_) -> None:
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
                # primary
                self.motion = motion_
                self.type = MType.TRANSITIVE
                self.poses = motion_.poses
                self.fps = motion_.fps
                self.num_of_frames: int = len(motion_.poses)
                self.time_length = self.num_of_frames * (1.0 / motion_.fps)

    ## TRANSITIVE ##
    motion_ = amass.load(
        file="data/fairmotion/amass_test_data/CMU/CMU/10/10_04_poses.npz",
        bm_path="data/fairmotion/amass_test_data/smplh/male/model.npz",
    )
    # all motions must have same fps for this implementation, so use first motion to set global
    fps = motion_.fps

    """
    [TESTING]
    start = 125
    offset = 0 + 1 - 3 + 6 + 2 + 2 + 2 + 1 + 1 + 1
    length = 145
    walk_to_walk = MotionData(motion_ops.cut(motion_, start + offset, start + length))
    """

    # Walk-to-walk cycle
    walk_to_walk = MotionData(motion_ops.cut(motion_, 300, 430), MType.TRANSITIVE)

    # Standing pose that must converted (amass -> habitat joint positions)
    standing_pose = motion_.poses[0]

    ## SCENIC ##
    motion_ = amass.load(
        file="data/fairmotion/amass_test_data/CMU/CMU/13/13_08_poses.npz",
        bm_path="data/fairmotion/amass_test_data/smplh/male/model.npz",
    )
    drink_beverage = MotionData(motion_, MType.SCENIC)


class PathData:
    """
    The PathData class is purposed to instantiate with given path points, and used
    to manage the path data fro the current path following sequence.
    """

    def __init__(self, path_points: List[mn.Vector3]) -> None:
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
        location: mn.Vector3,
        facing: Optional[mn.Vector3] = None,
    ) -> None:
        self.motion_data = motion_data
        self.location = mn.Vector3(location)
        self.facing = None
        if facing is not None:
            self.facing = mn.Vector3(facing)


# keeps track of the motion key frame preview modes
class Preview(Enum):
    OFF = 0
    KEYFRAMES = 1
    TRAJECTORY = 2
    ALL = 3


# keeps track of the activity that intances model is  participating in currently
class Activity(Enum):
    NONE = 0
    MOTION_FOLLOW = 1
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
    def check() -> None:
        """
        Records previous frame duration and updates the previous frame timestamp
        to the current time. If the timer is not currently running, perform nothing.
        """
        return time.time() - Timer.start_time
