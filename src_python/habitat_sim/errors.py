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
