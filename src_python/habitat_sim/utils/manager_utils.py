#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import csv


def save_csv_report(file_name: str, report_string: str):
    """This function takes a string, parses it on embedded newlines,
    and writes the resultant array of CSV strings to a file.  This is the output format
    of the various Attributes Managers and Physics Object manager's reports.

    :param file_name: The name of the file to write to.
    :param report_string: String with embedded newlines to par to write to file."""

    report_list = report_string.splitlines()

    with open(file_name, "w") as f:
        writer = csv.writer(f, quoting=csv.QUOTE_ALL)
        for x in report_list:
            writer.writerow(x.split(","))
