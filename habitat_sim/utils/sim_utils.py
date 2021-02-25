#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import habitat_sim

VisualSensorTypeSet = {
    habitat_sim.SensorType.COLOR,
    habitat_sim.SensorType.DEPTH,
    habitat_sim.SensorType.SEMANTIC,
}
CameraSensorSubTypeSet = {
    habitat_sim.SensorSubType.PINHOLE,
    habitat_sim.SensorSubType.ORTHOGRAPHIC,
}


def remove_all_objects(sim):
    r"""Removes all objects in simulator instance
    :param sim: Instance of Simulator
    """
    for obj_id in sim.get_existing_object_ids():
        sim.remove_object(obj_id)


def make_sensor_specs_from_settings(sensors, settings):
    r"""Creates and returns a list of initialized SensorSpecs
    :param sensors: Dictionary of sensors to initialize and relevant parameters, sensor_type and sensor_subtype are mandatory for each sensor
    :param settings: Dictionary modifying parameters and toggling sensors as true/false
    :return: List[hsim.SensorSpec] of initialized SensorSpecs
    """
    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        if settings[sensor_uuid]:
            # Check if type VisualSensorSpec
            if sensor_params["sensor_type"] not in VisualSensorTypeSet:
                # TODO: Add more checks for NonVisualSensorSpec
                raise ValueError(
                    f"""{sensor_params["sensor_type"]} is an illegal sensorType that is not implemented yet"""
                )
            else:
                # Check if type CameraSensorSpec
                if sensor_params["sensor_subtype"] in CameraSensorSubTypeSet:
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
                    if "silent" in settings and not settings["silent"]:
                        print("==== Initialized Sensor Spec: =====")
                        print("Sensor uuid: ", sensor_spec.uuid)
                        print("Sensor type: ", sensor_spec.sensor_type)
                        print("Sensor position: ", sensor_spec.position)
                        print("===================================")
                    sensor_specs.append(sensor_spec)
                # TODO: Add checks to initialize other types of SensorSpecs
    return sensor_specs


def make_sensor_specs(sensors):
    r"""Creates and returns a list of initialized SensorSpecs
    :param sensors: Dictionary of sensors to initialize and relevant parameters, sensor_type and sensor_subtype are mandatory for each sensor
    :return: List[hsim.SensorSpec] of initialized SensorSpecs
    """
    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        # Check if type VisualSensorSpec
        if sensor_params["sensor_type"] not in VisualSensorTypeSet:
            # TODO: Add more checks for NonVisualSensorSpec
            raise ValueError(
                f"""{sensor_params["sensor_type"]} is an illegal sensorType that is not implemented yet"""
            )
        else:
            # Check if type CameraSensorSpec
            if sensor_params["sensor_subtype"] in CameraSensorSubTypeSet:
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
            # TODO: Add checks to initialize other types of SensorSpecs
    return sensor_specs
