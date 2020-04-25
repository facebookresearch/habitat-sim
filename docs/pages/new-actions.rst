Add new actions
###############

:summary: This example shows how to add new actions *outside* of the source
    code.

Agent actions are implemented with "control" functors. These functors are
subclasses of the `habitat_sim.agent.SceneNodeControl` class and have a
:py:`__call__` method with the following signature:

.. code:: py

    def __call__(self,
            scene_node: habitat_sim.SceneNode,
            actuation_spec: habitat_sim.ActuationSpec):
        pass

The scene_node is what the control function manipulates (or controls) and the
actuation_spec contains any parameters needed by that control function.
See ``habitat_sim/agent/controls/default_controls.py`` for more example
controls.

Controls are registered using `habitat_sim.registry.register_move_fn()`.
This function takes the functor to register and, optionally, the name
to register it with, and whether or not the control effects the body or just
the sensors. If no name is given, the functor is registered with its own name,
converted to snake case.

This function can also be used as a decorator --- the following will register
the control functor ``MyNewControl`` with the name ``my_new_control``:

.. code:: py

    import habitat_sim

    @habitat_sim.registry.register_move_fn(body_action=True)
    class MyNewControl(habitat_sim.SceneNodeControl):
        pass

We also define two types of actions, body actions and non body actions. Whether
a control is a body or non body action is determined by whether it was
registered with :py:`body_action=True` or :py:`body_action=False`. Body actions
move the body of the agent (thereby also moving the sensors) while non-body
actions move just the sensors.

The example is runnable via

.. code:: shell-session

    $ python examples/tutorials/new_actions.py

.. include:: ../../examples/tutorials/new_actions.py
    :code: py
    :class: m-console-wrap
    :start-line: 6
