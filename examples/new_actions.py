#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import attr
import numpy as np
import quaternion

import habitat_sim
import habitat_sim.utils
from habitat_sim.agent import controls

try:
    import pprint

    _pprint = pprint.pprint
except ImportError:
    _pprint = print


r"""
This is a demonstration of how to add new actions to agents in habitat_sim

Agent actions are implemented with "control" functions.  These functions have
the following signature

def my_new_control(scene_node: habitat_sim.SceneNode, actuation_spec: habitat_sim.ActuationSpec):
    pass

The scene_node is what the control function manipulates (or controls) and the actuation_spec
contains any parameters needed by that control function.
See habitat_sim/agent/default_controls.py for example control functions

Control functions are registered use the habitat_sim.agent.controls.register_move_fn function.
This function takes the function to register and, optionally, the name to register it with.
If no name is given, the function is registered with its own name.

This function can also be used as a decorator
@controls.register_move_fn
def my_new_control(...):
    pass

will register the function my_new_control with the name my_new_control


We also define two types of actions.  BodyActions and non-BodyActions
BodyActions move the body of the agent (thereby also moving the sensors)
while non-BodyActions move just the sensors

To denote an action as a body action, it needs to be added
to the set of body actions: habitat_sim.agent.BodyActions
"""

# We will define an action that moves the agent and turns it by some amount

# First, define a class to keep the parameters of the control
# @attr.s is just syntatic sugar for creating these data-classes
@attr.s(auto_attribs=True, slots=True)
class MoveAndSpinSpec:
    forward_amount: float
    spin_amount: float


print(MoveAndSpinSpec(1.0, 45.0))


# Register the control function
# If done with the decorator sync, it will be registered with
# the same name of the function
@controls.register_move_fn
def move_forward_and_spin(
    scene_node: habitat_sim.SceneNode, actuation_spec: MoveAndSpinSpec
):
    forward_ax = scene_node.absolute_transformation()[0:3, 2]
    scene_node.translate_local(forward_ax * actuation_spec.forward_amount)

    # Rotate about the +y (up) axis
    rotation_ax = habitat_sim.geo.UP
    scene_node.rotate_local(np.deg2rad(actuation_spec.spin_amount), rotation_ax)
    # Calling normalize is needed after rotating to deal with machine precision errors
    scene_node.normalize()


# We can also register the function with a custom name
controls.register_move_fn(move_forward_and_spin, "my_custom_name")


# These actions are body actions, so we need to add them
print("Current body actions", habitat_sim.agent.BodyActions)
habitat_sim.agent.BodyActions.update({"my_custom_name", "move_forward_and_spin"})


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
backend_cfg.scene.id = "data/scene_datasets/habitat-test-scenes/van-gogh-room.glb"
sim = habitat_sim.Simulator(habitat_sim.Configuration(backend_cfg, [agent_config]))
print(sim.get_agent(0).state)


# Take the new actions!
sim.step("fwd_and_spin")
print(sim.get_agent(0).state)

sim.step("fwd_and_spin_double")
print(sim.get_agent(0).state)

sim.step(100)
print(sim.get_agent(0).state)

sim.close()
del sim


# Let's define a strafe action!


@attr.s(auto_attribs=True, slots=True)
class StrafeActuationSpec:
    forward_amount: float
    # Classic strafing is to move perpendicular (90 deg) to the forward direction
    strafe_angle: float = 90.0


def _strafe_impl(
    scene_node: habitat_sim.SceneNode, forward_amount: float, strafe_angle: float
):
    forward_ax = scene_node.absolute_transformation()[0:3, 2]
    rotation = habitat_sim.utils.quat_from_angle_axis(
        np.deg2rad(strafe_angle), habitat_sim.geo.UP
    )
    move_ax = habitat_sim.utils.quat_rotate_vector(rotation, forward_ax)
    print(move_ax)

    scene_node.translate_local(move_ax * forward_amount)


@controls.register_move_fn
def strafe_left(scene_node: habitat_sim.SceneNode, actuation_spec: StrafeActuationSpec):
    _strafe_impl(scene_node, actuation_spec.forward_amount, actuation_spec.strafe_angle)


@controls.register_move_fn
def strafe_right(
    scene_node: habitat_sim.SceneNode, actuation_spec: StrafeActuationSpec
):
    _strafe_impl(
        scene_node, actuation_spec.forward_amount, -actuation_spec.strafe_angle
    )


# These actions are body actions, so we need to add them
habitat_sim.agent.BodyActions.update({"strafe_right", "strafe_left"})

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
