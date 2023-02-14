# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import magnum as mn
import magnum.scenegraph


class InvalidAttachedObject(RuntimeError):
    pass


def assert_obj_valid(obj: mn.scenegraph.AbstractFeature3D) -> None:
    if not obj.object:
        raise InvalidAttachedObject(
            "Attached Object is invalid.  Attached to a valid scene graph before use."
        )


class GreedyFollowerError(RuntimeError):
    pass
