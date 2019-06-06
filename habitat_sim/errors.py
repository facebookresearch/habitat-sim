import habitat_sim.bindings as hsim


class InvalidAttachedObject(RuntimeError):
    pass


def assert_obj_valid(obj: hsim.AttachedObject):
    if not obj.object:
        raise InvalidAttachedObject(
            "Attached Object is invalid.  Attached to a valid scene graph before use."
        )


class GreedyFollowerError(RuntimeError):
    pass
