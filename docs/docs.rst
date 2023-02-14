..
    Stuff defined here gets set globally for everything else:

    -   use `thing` as a shortcut for :ref:`thing`
    -   use :py:`code` for inline code with highlighted Python syntax
..

.. default-role:: ref

.. role:: py(code)
    :language: py

.. attrs __init__ function docs go here

.. TODO: what is _try_step?!
.. py:function:: habitat_sim.agent.ObjectControls.__init__
    :summary: Constructor
    :param move_filter_fn: A function that is applied after actions to handle
        collisions, This should generally be `nav.PathFinder.try_step`

.. docs for bindings go here -- doing all the formatting in a C++ raw string is
    worse than a hangover

.. docs NOTE: a detail string is required to generate the entry. Only populating params, return, and summary will be skipped...

.. py:class:: habitat_sim.scene.SemanticCategory
    :summary: Base class for all semantic categories.

    Different datasets and semantic types (i.e. object or region) will have
    different information, so there are specific implementations, i.e.
    `Mp3dObjectCategory` and `Mp3dRegionCategory`.

.. py:property:: habitat_sim.scene.SemanticRegion.id

    For some datasets, ``<region_id>`` portion of the full ID will be globally
    unique, while in other datasets, it will only be unique for each level.

.. py:property:: habitat_sim.scene.SemanticObject.id

    For some datasets, ``<region_id>`` portion of the full ID will be globally
    unique, while in other datasets, it will only be unique for each level.

.. py:function:: habitat_sim.gfx.RenderTarget.read_frame_rgba
    :summary: Reads RGBA frame into passed img in uint8 byte format

    The argument is a `numpy.ndarray` to populate with frame bytes. Memory is
    *not* allocated to this array. Assume that :py:`m = height` and
    :py:`n = width * 4`.

.. py:function:: habitat_sim.nav.PathFinder.distance_to_closest_obstacle
    :summary: Returns the distance to the closest obstacle

    If this distance is greater than :p:`max_search_radius`,
    :p:`max_search_radius` is returned instead.

.. py:function:: habitat_sim.nav.PathFinder.closest_obstacle_surface_point
    :summary: Returns the `hit_pos <HitRecord.hit_pos>`,
        `hit_normal <HitRecord.hit_normal>`, and
        `hit_dist <HitRecord.hit_dist>` of the surface point on the closest
        obstacle.

    If the returned `hit_dist <HitRecord.hit_dist>` is equal to
    :p:`max_search_radius`, no obstacle was found.

.. py:function:: habitat_sim.nav.PathFinder.is_navigable
    :summary: Checks to see if the agent can stand at the specified point

    To check navigability, the point is snapped to the nearest polygon and then
    the snapped point is compared to the original point. Any amount of x-z
    translation indicates that the given point is not navigable. The amount of
    y-translation allowed is specified by :p:`max_y_delta` to account for
    slight differences in floor height.

.. py:function:: habitat_sim.nav.PathFinder.try_step
    :summary: Find a valid location for the agent to actually step to when it attempts to step between start and end

    Attempts to move from start to end and returns the navigable point closest to end that is feasibly reachable from start.

    :param start: The starting location of the agent
    :param end: The desired end location
    :return: The actual ending location, if such a location exists, or ``{NAN, NAN, NAN}``

.. py:function:: habitat_sim.nav.PathFinder.get_random_navigable_point
    :summary: Samples a navigable point uniformly at random from the navmesh

    This method can fail.  If it does, the returned point will be `{NAN, NAN, NAN}`. Use is_navigable to check if the point is navigable.

    :param max_tries: The maximum number of times to retry sampling if it fails and the navmesh seems fine. Setting this higher can sometimes be warranted, but needing to typically indicates an error with the navmesh.
    :param island_index: Optionally specify the island from which to sample the point. Default -1 queries the full navmesh.
    :return: A navigable point or ``{NAN, NAN, NAN}`` if this fails

.. py:function:: habitat_sim.nav.PathFinder.snap_point
    :summary: Snaps a point to the closet navigable location

    Will only search within a 4x8x4 cube centerred around the point.
    If there is no navigable location within that cube, no navigable point will be found.

    :param point: The starting location of the agent
    :param island_index: Optionally specify the island from which to sample the point. Default -1 queries the full navmesh.
    :return: The navigable point, if one exists, or ``{NAN, NAN, NAN}``

.. dump of whatever else was in the other PR

.. py:module:: habitat_sim.agent

    See also `AgentConfiguration`, `AgentState` and `SixDOFPose` for more
    information.

    Actions
    =======

    We currently have the following actions added by default. Any action not
    registered with an explict name is given the snake case version of the
    class name, i.e. ``MoveForward`` can be accessed with the name
    ``move_forward``.  See `registry.register_move_fn`, `SceneNodeControl`,
    and `ActuationSpec`

    .. include:: ../src_python/habitat_sim/agent/controls/default_controls.py
        :code: py
        :start-line: 38

    And noisy actions from PyRobot.  See `PyRobotNoisyActuationSpec`

    .. include:: ../src_python/habitat_sim/agent/controls/pyrobot_noisy_controls.py
        :code: py
        :start-line: 244


    Action space path finding
    =========================

    See the `nav.GreedyGeodesicFollower` class.

.. py:module:: habitat_sim.simulator

    Core
    ====

    See `Simulator`, `Configuration` and `sim.SimulatorConfiguration`.

    Semantic Scene
    ==============

    The Semantic scene provides access to semantic information about the given
    environement

    .. note-warning::

        Not avaliable for all datasets.

.. py:module:: habitat_sim.utils.common

    Quaternion Math
    ===============

    Quaternion helper functions:

    -   `quat_from_coeffs()`
    -   `quat_to_coeffs()`
    -   `quat_from_angle_axis()`
    -   `quat_to_angle_axis()`
    -   `quat_from_two_vectors()`
    -   `angle_between_quats()`
    -   `quat_rotate_vector()`

    Misc
    ====

    -   `colorize_ids()`

.. py:data:: habitat_sim.utils.common.d3_40_colors_rgb
    :summary: Color map for semantic ID rendering.

.. py:data:: habitat_sim.utils.common.d3_40_colors_hex
    :summary: Color map for semantic ID rendering.

    Same as `d3_40_colors_rgb`, but in a hexadecimal representation.

    .. include:: ../src_python/habitat_sim/utils/common.py
        :code: py
        :start-after: # [d3_40_colors_hex]
        :end-before: # [/d3_40_colors_hex]
        :filters: string_hex_colors

.. py:data:: habitat_sim.utils.settings.default_sim_settings
    :summary: Quickstart settings dictionary. Can be passed directly to settings.make_cfg() to create a default Configuration for an empty scene. Edit to customize the simulator.

    .. include:: ../src_python/habitat_sim/utils/settings.py
        :code: py
        :start-after: # [default_sim_settings]
        :end-before: # [/default_sim_settings]
