import habitat_sim.gfx.LightInfo
import habitat_sim.gfx.NO_LIGHT_KEY


def test_get_no_light_setup(sim):
    light_setup = sim.get_light_setup(habitat_sim.gfx.NO_LIGHT_KEY)

    assert len(light_setup) == 0


def test_set_default_light_setup(sim):
    light_setup = [LightInfo(position=[1.0, 1.0, 1.0])]

    sim.set_light_setup(light_setup)

    assert sim.get_light_setup() == light_setup


def test_set_custom_light_setup(sim):
    custom_setup_key = "custom_setup_key"
    light_setup = sim.get_light_setup(custom_setup_key)
    assert len(light_setup) == 0

    light_setup.append(LightInfo(position=[1.0, 1.0, 1.0]))

    sim.set_light_setup(light_setup, custom_setup_key)

    assert sim.get_light_setup(custom_setup_key) == light_setup
