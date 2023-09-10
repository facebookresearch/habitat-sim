# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

r"""
Simple example showing how to overlap physics and rendering in Habitat-Sim.  The general
pattern is to call sim.start_async_render_and_step_physics instead of sim.step_physics for
the *first* step_physics call and then retrieve observations using sim.get_sensor_observations_async_finish
instead of sim.get_sensor_observations


Known limitations/issues:
    * Creating and destroying simulation instances leaks GPU memory.  On some drivers
    OpenGL leaks GPU memory when the background thread used for rendering is destroyed.
    The workaround is to use sim.close(destroy=False) then sim.reconfigure(new_cfg) instead of
    sim = habitat_sim.Simulator(new_cfg) as this will keep the background thread alive.

    * Segfaults with headed Linux builds.  We've seen this segfault for headed builds.  There
    isn't any reason why the headed Linux build should segfault so we've left it enabled
    under that this is a driver bug and it may work with other driver/OS combinations.

    * Semantic Sensor rendering does not work when there is an RGB mesh and a semantic mesh, like
    in MP3D and Gibson.  When there is a single mesh for RGB and semantics, a semantic sensor works.

    * Sensors that aren't CameraSensors, i.e. FisheyeSensor or EquirectangularSensor, are not
    supported

    * Adding or deleting objects during the async render is not supported.  This will lead to
    an assert checking that the main process owns the OpenGL failing.  You can wait on the
    async render to finish and transfer the context back to the main thread by calling
    sim.renderer.acquire_gl_context().  Note that by default the OpenGL context is transferred
    back to the main thread by calling sim.get_sensor_observations_async_finish() by default.
"""

import habitat_sim


def main():
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = (
        "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
    )

    # Leaving the context with the background thread can improve
    # performance, but makes it so that sim.get_sensor_observations_async_finish()
    # no longer returns the context back to the main thread. If you set this to
    # true and get context assert failures, call sim.renderer.acquire_gl_context()
    # to move the OpenGL context back to the main thread.
    # The default is False
    backend_cfg.leave_context_with_background_renderer = False

    agent_cfg = habitat_sim.AgentConfiguration(
        sensor_specifications=[habitat_sim.CameraSensorSpec()]
    )
    cfg = habitat_sim.Configuration(backend_cfg, [agent_cfg])

    # Number of physics steps per render
    n_physics_steps_per_render = 4
    # Render at 30 hz
    physics_step_time = 1.0 / (30 * n_physics_steps_per_render)

    sim = habitat_sim.Simulator(cfg)

    for i in range(n_physics_steps_per_render):
        # Always call sim.step_physics when not using async rendering
        if i == 0:
            sim.start_async_render_and_step_physics(physics_step_time)
        else:
            sim.step_physics(physics_step_time)

        # Call sim.renderer.acquire_gl_context() if you need to create or destroy an object.
        # Note that this will block until rendering is done.

    obs = sim.get_sensor_observations_async_finish()
    # To change back to normal rendering, use
    #     obs = sim.get_sensor_observations()

    # You can add/remove objects after the call to get_sensor_observations_async_finish()
    # if backend_cfg.leave_context_with_background_renderer was left as False, it that as
    # true, you'd need to call
    #     sim.renderer.acquire_gl_context()
    # Calling acquire_gl_context() is a noop if the main thread already has the OpenGL context

    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = "data/scene_datasets/habitat-test-scenes/apartment_1.glb"
    new_cfg = habitat_sim.Configuration(backend_cfg, [agent_cfg])

    # To create a new instance of the simulator to swap scene or similar,
    # do not delete the simulator instance as this may leak GPU memory
    # Instead, call sim.close(destroy=False) as this will close/delete
    # everything except the OpenGL context and the background render
    # thread.
    sim.close(destroy=False)
    # Then use reconfigure.
    sim.reconfigure(new_cfg)
    # If
    #     sim.close()
    #     sim = habitat_sim.Simulator(new_cfg)
    # was used, that could end up leaking memory

    sim.start_async_render_and_step_physics(physics_step_time)
    obs = sim.get_sensor_observations_async_finish()  # noqa: F841

    # Call close with destroy=True here because this example is over :)
    sim.close(destroy=True)


if __name__ == "__main__":
    main()
