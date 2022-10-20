# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import csv
import datetime
import os
from enum import Enum
from typing import List

import magnum as mn

# A line of dashes to divide sections of terminal output
section_divider: str = "\n" + "-" * 72

# globals to dictate if we are logging anything to console.
# Are overriden from config file
silent = True
debug_print = False


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


class RotationAxis(Enum):
    Y = 0
    X = 1


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
