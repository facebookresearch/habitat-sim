#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import pytest

# Need to import quaternion library here despite it not being used or else importing
# habitat_sim below will cause an invalid free() when audio is enabled in sim compilation
import quaternion  # noqa: F401

from habitat_sim.registry import _camel_to_snake, _Registry, registry

# -- _camel_to_snake tests --


class TestCamelToSnake:
    """Tests for the _camel_to_snake helper function."""

    def test_simple_camel_case(self):
        assert _camel_to_snake("MoveForward") == "move_forward"

    def test_single_word(self):
        assert _camel_to_snake("Move") == "move"

    def test_already_snake_case(self):
        assert _camel_to_snake("move_forward") == "move_forward"

    def test_all_lowercase(self):
        assert _camel_to_snake("move") == "move"

    def test_multiple_capitals(self):
        assert _camel_to_snake("MoveForwardFast") == "move_forward_fast"

    def test_consecutive_capitals(self):
        """Consecutive capitals like 'HTML' should be handled."""
        result = _camel_to_snake("HTMLParser")
        assert result == "html_parser"

    def test_single_char_words(self):
        assert _camel_to_snake("AClass") == "a_class"

    def test_numbers_in_name(self):
        assert _camel_to_snake("Move3D") == "move3_d"

    def test_empty_string(self):
        assert _camel_to_snake("") == ""

    @pytest.mark.parametrize(
        "input_name,expected",
        [
            ("LookUp", "look_up"),
            ("LookDown", "look_down"),
            ("LookLeft", "look_left"),
            ("LookRight", "look_right"),
            ("TurnLeft", "turn_left"),
            ("TurnRight", "turn_right"),
            ("MoveBackward", "move_backward"),
        ],
    )
    def test_common_control_names(self, input_name, expected):
        """Common habitat-sim control class names convert correctly."""
        assert _camel_to_snake(input_name) == expected


# -- _Registry tests --


class TestRegistry:
    """Tests for the _Registry class and the global registry instance."""

    def test_global_registry_is_instance(self):
        assert isinstance(registry, _Registry)

    def test_get_move_fn_returns_none_for_unknown(self):
        """Querying an unregistered move_fn should return None."""
        assert registry.get_move_fn("nonexistent_action_12345") is None

    def test_get_noise_model_returns_none_for_unknown(self):
        """Querying an unregistered noise model should return None."""
        assert registry.get_noise_model("nonexistent_model_12345") is None

    def test_mapping_is_defaultdict(self):
        """Accessing a new key in _mapping should return empty dict, not KeyError."""
        result = _Registry._mapping["some_new_category_xyz"]
        assert result == {}

    def test_get_impl_returns_none_for_empty_category(self):
        assert _Registry._get_impl("totally_empty_category_abc", "anything") is None

    def test_register_move_fn_requires_body_action(self):
        """register_move_fn must have body_action explicitly set."""
        with pytest.raises(AssertionError, match="body_action"):
            registry.register_move_fn(name="test_action")

    def test_registered_move_fns_are_retrievable(self):
        """Default controls should be registered and retrievable.

        The default_controls module registers standard movement controls
        on import. Verify at least one exists.
        """
        import habitat_sim.agent.controls.default_controls  # noqa: F401

        fn = registry.get_move_fn("move_forward")
        assert fn is not None

    def test_registered_noise_models_are_retrievable(self):
        """Built-in noise models should be registered and retrievable."""
        import habitat_sim.sensors.noise_models  # noqa: F401

        model = registry.get_noise_model("None")
        assert model is not None

    def test_register_noise_model_decorator_no_name(self):
        """register_noise_model with no name uses class name."""
        from habitat_sim.sensors.noise_models.sensor_noise_model import SensorNoiseModel

        @registry.register_noise_model
        class _TestNoiseModel123(SensorNoiseModel):
            @staticmethod
            def apply(x):
                return x

        assert registry.get_noise_model("_TestNoiseModel123") is not None

    def test_register_noise_model_decorator_with_name(self):
        """register_noise_model with explicit name uses that name."""
        from habitat_sim.sensors.noise_models.sensor_noise_model import SensorNoiseModel

        @registry.register_noise_model(name="custom_test_noise_456")
        class _AnotherTestNoise(SensorNoiseModel):
            @staticmethod
            def apply(x):
                return x

        assert registry.get_noise_model("custom_test_noise_456") is not None
