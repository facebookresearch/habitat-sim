#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import habitat_sim


def remove_all_objects(sim):
    for obj_id in sim.get_existing_object_ids():
        sim.remove_object(obj_id)


def make_sensor_specs_from_settings(sensors, settings):
    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        if settings[sensor_uuid]:
            # Check if type VisualSensorSpec
            if (
                sensor_params["sensor_type"] == habitat_sim.SensorType.COLOR
                or sensor_params["sensor_type"] == habitat_sim.SensorType.DEPTH
                or sensor_params["sensor_type"] == habitat_sim.SensorType.SEMANTIC
            ):
                # Check if type CameraSensorSpec
                if (
                    sensor_params["sensor_subtype"] == habitat_sim.SensorSubType.PINHOLE
                    or sensor_params["sensor_subtype"]
                    == habitat_sim.SensorSubType.ORTHOGRAPHIC
                ):
                    sensor_spec = habitat_sim.CameraSensorSpec()
                    sensor_spec.uuid = sensor_uuid
                    sensor_spec.sensor_type = sensor_params["sensor_type"]
                    sensor_spec.sensor_subtype = sensor_params["sensor_subtype"]
                    if "resolution" in sensor_params:
                        sensor_spec.resolution = sensor_params["resolution"]
                    if "position" in sensor_params:
                        sensor_spec.position = sensor_params["position"]
                    if "orientation" in sensor_params:
                        sensor_spec.position = sensor_params["orientation"]
                    sensor_spec.gpu2gpu_transfer = False
                    if not settings["silent"]:
                        print("==== Initialized Sensor Spec: =====")
                        print("Sensor uuid: ", sensor_spec.uuid)
                        print("Sensor type: ", sensor_spec.sensor_type)
                        print("Sensor position: ", sensor_spec.position)
                        print("===================================")
                    sensor_specs.append(sensor_spec)
                else:
                    # TODO: Add more checks for other types of sensors
                    continue
            else:
                # TODO: Add more checks for NonVisualSensorSpec
                continue
    return sensor_specs


def make_sensor_specs(sensors):
    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        # Check if type VisualSensorSpec
        if (
            sensor_params["sensor_type"] == habitat_sim.SensorType.COLOR
            or sensor_params["sensor_type"] == habitat_sim.SensorType.DEPTH
            or sensor_params["sensor_type"] == habitat_sim.SensorType.SEMANTIC
        ):
            # Check if type CameraSensorSpec
            if (
                sensor_params["sensor_subtype"] == habitat_sim.SensorSubType.PINHOLE
                or sensor_params["sensor_subtype"]
                == habitat_sim.SensorSubType.ORTHOGRAPHIC
            ):
                sensor_spec = habitat_sim.CameraSensorSpec()
                sensor_spec.uuid = sensor_uuid
                sensor_spec.sensor_type = sensor_params["sensor_type"]
                sensor_spec.sensor_subtype = sensor_params["sensor_subtype"]
                if "resolution" in sensor_params:
                    sensor_spec.resolution = sensor_params["resolution"]
                if "position" in sensor_params:
                    sensor_spec.position = sensor_params["position"]
                if "orientation" in sensor_params:
                    sensor_spec.position = sensor_params["orientation"]
                sensor_spec.gpu2gpu_transfer = False
                sensor_specs.append(sensor_spec)
            else:
                # TODO: Add more checks for other types of sensors
                continue
        else:
            # TODO: Add more checks for NonVisualSensorSpec
            continue
    return sensor_specs
