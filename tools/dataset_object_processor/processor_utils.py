# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import csv
import datetime
import os
from enum import Enum
from typing import List, Tuple

import magnum as mn

# A line of dashes to divide sections of terminal output
section_divider: str = "\n" + "-" * 72

# globals to dictate if we are logging anything to console.
# Are overriden from config file
silent = True
debug_print = False


class RotationAxis(Tuple, Enum):
    Y = (0.0, 1.0, 0.0)
    X = (1.0, 0.0, 0.0)


class ANSICodes(Enum):
    """
    Terminal printing ANSI color and format codes
    """

    HEADER = "\033[95m"
    BROWN = "\033[38;5;130m"
    ORANGE = "\033[38;5;202m"
    YELLOW = "\033[38;5;220m"
    PURPLE = "\033[38;5;177m"
    BRIGHT_RED = "\033[38;5;196m"
    BRIGHT_BLUE = "\033[38;5;27m"
    BRIGHT_MAGENTA = "\033[38;5;201m"
    BRIGHT_CYAN = "\033[38;5;14m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    BOLD = "\033[1m"
    ITALIC = "\033[3m"
    UNDERLINE = "\033[4m"


class CSVWriter:
    """
    Generalized utility to write csv files
    """

    def write_file(
        headers: List[str],
        csv_rows: List[List[str]],
        file_path: str,
    ) -> None:
        """
        Write column titles and csv data into csv file with the provided
        file path.
        :param headers: List of strings that refers to csv column titles
        :param csv_rows: A List of a List of strings, where each List of strings
        is a row of the csv file, one string entry for each column
        :param file_path: absolute file path of csv file to save to
        """
        # make sure the number of columns line up
        if not len(csv_rows[0]) == len(headers):
            raise RuntimeError(
                "Number of headers does not equal number of columns in CSVWriter.write_file()."
            )

        with open(file_path, "w") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(headers)
            writer.writerows(csv_rows)


class MemoryUnitConverter:
    """
    class to convert computer memory value units, i.e.
    1,024 bytes to 1 kilobyte, or (1 << 10) bytes
    1,048,576 bytes to 1 megabyte, or (1 << 20) bytes
    1,073,741,824 bytes to 1 gigabyte, or (1 << 30) bytes
    """

    BYTES = 0
    KILOBYTES = 1
    MEGABYTES = 2
    GIGABYTES = 3

    UNIT_STRS = ["bytes", "KB", "MB", "GB"]
    UNIT_CONVERSIONS = [1, 1 << 10, 1 << 20, 1 << 30]


def print_if_logging(message: str = "") -> None:
    """
    Print to console if "silent" is set to false in the config file
    """
    if not silent:
        print(message)


def print_debug(message: str = "") -> None:
    """
    Print to console if "debug_print" is set to true in the config file
    """
    if debug_print:
        print(message)


def print_quaternion_debug(name: str, q: mn.Quaternion, color) -> None:
    """
    Print quaternion to console in angle-axis form if "debug_print" is set
    to true in the config file
    """
    global debug_print
    if debug_print:
        decimal = 1
        angle = round(float(mn.Deg(q.angle())), decimal)
        x = round(q.axis().x, decimal)
        y = round(q.axis().y, decimal)
        z = round(q.axis().z, decimal)
        print(color + name + f"{angle} ({x}, {y}, {z})")


def print_mem_usage_info(
    start_mem,
    end_mem,
    avg_ram_used_str: str,
) -> None:
    """"""
    # Print memory usage info before loading object
    text_format = ANSICodes.BRIGHT_RED.value
    print_if_logging(text_format + "\nstart mem state" + section_divider)
    for key, value in start_mem.items():
        value_str = value
        if key != "percent":
            value_str = get_mem_size_str(value)
        print_if_logging(text_format + f"{key} : {value_str}")

    # Print memory usage info after loading object
    print_if_logging(text_format + "\nend mem state" + section_divider)
    for key, value in end_mem.items():
        value_str = value
        if key != "percent":
            value_str = get_mem_size_str(value)
        print_if_logging(text_format + f"{key} : {value_str}")

    # Print difference in memory usage before and after loading object
    print_if_logging(text_format + "\nchange in mem states" + section_divider)
    for (key_s, value_s), (key_e, value_e) in zip(start_mem.items(), end_mem.items()):
        value_str = value_e - value_s
        if key_s != "percent" and key_e != "percent":
            value_str = get_mem_size_str(value_e - value_s)
        print_if_logging(text_format + f"{key_s} : {value_str}")

    # Print rough estimate of RAM used when loading object
    print_if_logging(
        text_format + "\naverage RAM used" + section_divider + f"\n{avg_ram_used_str}"
    )


def create_unique_filename(
    dir_path: str, extension: str, filename_prefix: str = None
) -> str:
    """
    Create unique file name / file path based off of the current date and time.
    Also create directory in which we save the file if one doesn't already exist
    :param dir_path: Absolute file path of directory in which to create this new
    file
    :param extension: extension of file name. Of the form ".mp4", ".csv", etc
    :param filename_prefix: if you want the filename to be more descriptive,
    rather than just have the date and time. The filepath will be of the form:
    <path to dir>/<filename_prefix>__date_<year>-<month>-<day>__time_<hour>:<min>:<sec>.
    If no filename_prefix: <path to dir>/date_<year>-<month>-<day>__time_<hour>:<min>:<sec>
    """
    # Current date and time so we can make unique file names for each csv
    date_and_time = datetime.datetime.now()

    # year-month-day
    date = date_and_time.strftime("%Y-%m-%d")

    # hour:min:sec - capital H is military time, %I is standard time
    # (am/pm time format)
    time = date_and_time.strftime("%H:%M:%S")

    # make directory to store file if it doesn't exist
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)

    # adjust prefix to make sure there is a delimeter between it and the rest
    # of the file name
    if filename_prefix is None or filename_prefix == "":
        filename_prefix = ""
    else:
        filename_prefix = f"{filename_prefix}__"

    # make sure directory path and filename extension are formatted correctly
    if not dir_path.endswith("/"):
        dir_path = dir_path + "/"
    if not extension.startswith("."):
        extension = "." + extension

    # create file name
    file_path = f"{dir_path}{filename_prefix}date_{date}__time_{time}{extension}"
    return file_path


def convert_memory_units(
    size: float,
    unit_type: int = MemoryUnitConverter.KILOBYTES,
    decimal_num_round: int = 2,
) -> float:
    """
    Convert units from bytes to desired unit, then round the result
    """
    new_size = size / MemoryUnitConverter.UNIT_CONVERSIONS[unit_type]
    return round(new_size, decimal_num_round)


def get_mem_size_str(
    size: float,
    unit_type: int = MemoryUnitConverter.KILOBYTES,
) -> str:
    """
    Convert bytes to desired memory unit size, then create string
    that will be written into csv file rows. Add commas to numbers
    """
    new_size: float = convert_memory_units(size, unit_type)
    new_size_str: str = "{:,}".format(new_size)
    unit_str: str = MemoryUnitConverter.UNIT_STRS[unit_type]
    return f"{new_size_str} {unit_str}"


def get_bounding_box_corners(obj) -> List[mn.Vector3]:
    """
    Return a list of object bounding box corners in object local space.
    :param obj: a physics.ManagedBulletObject
    """
    bounding_box = obj.root_scene_node.cumulative_bb
    return [
        bounding_box.back_bottom_left,
        bounding_box.back_bottom_right,
        bounding_box.back_top_right,
        bounding_box.back_top_left,
        bounding_box.front_top_left,
        bounding_box.front_top_right,
        bounding_box.front_bottom_right,
        bounding_box.front_bottom_left,
    ]


def get_csv_headers(sim) -> List[str]:
    """
    Collect the csv column titles we'll need given which tests we ran
    """
    headers: List[str] = sim.sim_settings["object_name"]
    data_to_collect = sim.sim_settings["data_to_collect"]
    if data_to_collect.get("memory_data"):
        headers += sim.sim_settings["memory_data_headers"]
    if data_to_collect.get("render_time_ratio"):
        headers += sim.sim_settings["render_time_headers"]
    if data_to_collect.get("physics_data"):
        headers += sim.sim_settings["physics_data_headers"]

    return headers


def create_csv_file(
    headers: List[str],
    csv_rows: List[List[str]],
    csv_dir_path: str = None,
    csv_file_prefix: str = None,
) -> None:
    """
    Set directory where our csv's will be saved, create the csv file name,
    create the column names of our csv data, then open and write the csv
    file
    :param headers: column titles of csv file
    :param csv_rows: List of Lists of strings defining asset processing results
    for each dataset object
    :param csv_dir_path: absolute path to directory where csv file will be saved
    :param csv_file_prefix: prefix we will add to beginning of the csv filename
    to specify which dataset this csv is describing
    """
    file_path = create_unique_filename(csv_dir_path, ".csv", csv_file_prefix)

    text_format = ANSICodes.PURPLE.value + ANSICodes.BOLD.value
    print_if_logging(text_format + "\nWriting csv results to:" + section_divider)
    text_format = ANSICodes.PURPLE.value
    print_if_logging(text_format + f"{file_path}\n")

    CSVWriter.write_file(headers, csv_rows, file_path)

    text_format = ANSICodes.PURPLE.value
    print_if_logging(text_format + "CSV writing done\n")
