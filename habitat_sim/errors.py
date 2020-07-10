import magnum as mn
import magnum.scenegraph

from habitat_sim import bindings as hsim


class InvalidAttachedObject(RuntimeError):
    pass


def assert_obj_valid(obj: mn.scenegraph.AbstractFeature3D):
    if not obj.object:
        raise InvalidAttachedObject(
            "Attached Object is invalid.  Attached to a valid scene graph before use."
        )


class GreedyFollowerError(RuntimeError):
    pass
