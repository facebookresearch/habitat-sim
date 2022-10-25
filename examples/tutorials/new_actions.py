#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import attr
import magnum as mn
import numpy as np
import quaternion  # noqa: F401

import habitat_sim
from habitat_sim.utils.common import quat_from_angle_axis, quat_rotate_vector

try:
    import pprint

    _pprint = pprint.pprint
except ImportError:
    _pprint = print


# This is wrapped in a such that it can be added to a unit test
def main():
    # We will define an action that moves the agent and turns it by some amount

    # First, define a class to keep the parameters of the control
    # @attr.s is just syntatic sugar for creating these data-classes
    @attr.s(auto_attribs=True, slots=True)
    class MoveAndSpinSpec:
        forward_amount: float
        spin_amount: float

    print(MoveAndSpinSpec(1.0, 45.0))

    # Register the control functor
    # This action will be an action that effects the body, so body_action=True
    @habitat_sim.registry.register_move_fn(body_action=True)
    class MoveForwardAndSpin(habitat_sim.SceneNodeControl):
        def __call__(
            self, scene_node: habitat_sim.SceneNode, actuation_spec: MoveAndSpinSpec
        ):
            forward_ax = (
                np.array(scene_node.absolute_transformation().rotation_scaling())
                @ habitat_sim.geo.FRONT
            )
            scene_node.translate_local(forward_ax * actuation_spec.forward_amount)

            # Rotate about the +y (up) axis
            rotation_ax = habitat_sim.geo.UP
            scene_node.rotate_local(mn.Deg(actuation_spec.spin_amount), rotation_ax)
            # Calling normalize is needed after rotating to deal with machine precision errors
            scene_node.rotation = scene_node.rotation.normalized()

    # We can also register the function with a custom name
    habitat_sim.registry.register_move_fn(
        MoveForwardAndSpin, name="my_custom_name", body_action=True
    )

    # We can also re-register this function such that it effects just the sensors
    habitat_sim.registry.register_move_fn(
        MoveForwardAndSpin, name="move_forward_and_spin_sensors", body_action=False
    )

    # Now we need to add this action to the agent's action space in the configuration!
    agent_config = habitat_sim.AgentConfiguration()
    _pprint(agent_config.action_space)

    # We can add the control function in a bunch of ways
    # Note that the name of the action does not need to match the name the control function
    # was registered under.

    # The habitat_sim.ActionSpec defines an action.  The first arguement is the regsitered name
    # of the control spec, the second is the parameter spec
    agent_config.action_space["fwd_and_spin"] = habitat_sim.ActionSpec(
        "move_forward_and_spin", MoveAndSpinSpec(1.0, 45.0)
    )

    # Add the sensor version
    agent_config.action_space["fwd_and_spin_sensors"] = habitat_sim.ActionSpec(
        "move_forward_and_spin_sensors", MoveAndSpinSpec(1.0, 45.0)
    )

    # Add the same control with different parameters
    agent_config.action_space["fwd_and_spin_double"] = habitat_sim.ActionSpec(
        "move_forward_and_spin", MoveAndSpinSpec(2.0, 90.0)
    )

    # Use the custom name with an integer name for the action
    agent_config.action_space[100] = habitat_sim.ActionSpec(
        "my_custom_name", MoveAndSpinSpec(0.1, 1.0)
    )

    _pprint(agent_config.action_space)

    # Spin up the simulator
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = "data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"
    sim = habitat_sim.Simulator(habitat_sim.Configuration(backend_cfg, [agent_config]))
    print(sim.get_agent(0).state)

    # Take the new actions!
    sim.step("fwd_and_spin")
    print(sim.get_agent(0).state)

    # Take the new actions!
    sim.step("fwd_and_spin_sensors")
    print(sim.get_agent(0).state)

    sim.step("fwd_and_spin_double")
    print(sim.get_agent(0).state)

    sim.step(100)
    print(sim.get_agent(0).state)

    sim.close()

    # Let's define a strafe action!
    @attr.s(auto_attribs=True, slots=True)
    class StrafeActuationSpec:
        forward_amount: float
        # Classic strafing is to move perpendicular (90 deg) to the forward direction
        strafe_angle: float = 90.0

    def _strafe_impl(
        scene_node: habitat_sim.SceneNode, forward_amount: float, strafe_angle: float
    ):
        forward_ax = (
            np.array(scene_node.absolute_transformation().rotation_scaling())
            @ habitat_sim.geo.FRONT
        )
        rotation = quat_from_angle_axis(np.deg2rad(strafe_angle), habitat_sim.geo.UP)
        move_ax = quat_rotate_vector(rotation, forward_ax)

        scene_node.translate_local(move_ax * forward_amount)

    @habitat_sim.registry.register_move_fn(body_action=True)
    class StrafeLeft(habitat_sim.SceneNodeControl):
        def __call__(
            self, scene_node: habitat_sim.SceneNode, actuation_spec: StrafeActuationSpec
        ):
            _strafe_impl(
                scene_node, actuation_spec.forward_amount, actuation_spec.strafe_angle
            )

    @habitat_sim.registry.register_move_fn(body_action=True)
    class StrafeRight(habitat_sim.SceneNodeControl):
        def __call__(
            self, scene_node: habitat_sim.SceneNode, actuation_spec: StrafeActuationSpec
        ):
            _strafe_impl(
                scene_node, actuation_spec.forward_amount, -actuation_spec.strafe_angle
            )

    agent_config = habitat_sim.AgentConfiguration()
    agent_config.action_space["strafe_left"] = habitat_sim.ActionSpec(
        "strafe_left", StrafeActuationSpec(0.25)
    )
    agent_config.action_space["strafe_right"] = habitat_sim.ActionSpec(
        "strafe_right", StrafeActuationSpec(0.25)
    )

    sim = habitat_sim.Simulator(habitat_sim.Configuration(backend_cfg, [agent_config]))
    print(sim.get_agent(0).state)

    sim.step("strafe_left")
    print(sim.get_agent(0).state)

    sim.step("strafe_right")
    print(sim.get_agent(0).state)


if __name__ == "__main__":
    main()
