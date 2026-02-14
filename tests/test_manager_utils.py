#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import csv
import os

from habitat_sim.utils.manager_utils import save_csv_report


class TestSaveCsvReport:
    """Tests for the save_csv_report function."""

    def test_creates_file(self, tmp_path):
        path = str(tmp_path / "report.csv")
        save_csv_report(path, "a,b,c")
        assert os.path.isfile(path)

    def test_single_line(self, tmp_path):
        path = str(tmp_path / "report.csv")
        save_csv_report(path, "col1,col2,col3")
        with open(path) as f:
            reader = csv.reader(f)
            rows = list(reader)
        assert len(rows) == 1
        assert rows[0] == ["col1", "col2", "col3"]

    def test_multiple_lines(self, tmp_path):
        path = str(tmp_path / "report.csv")
        save_csv_report(path, "header1,header2\nval1,val2\nval3,val4")
        with open(path) as f:
            reader = csv.reader(f)
            rows = list(reader)
        assert len(rows) == 3
        assert rows[0] == ["header1", "header2"]
        assert rows[1] == ["val1", "val2"]
        assert rows[2] == ["val3", "val4"]

    def test_empty_string(self, tmp_path):
        """Empty string produces an empty file (splitlines returns [])."""
        path = str(tmp_path / "report.csv")
        save_csv_report(path, "")
        with open(path) as f:
            content = f.read()
        assert content == ""

    def test_quoted_output(self, tmp_path):
        """All fields should be quoted (csv.QUOTE_ALL)."""
        path = str(tmp_path / "report.csv")
        save_csv_report(path, "a,b")
        with open(path) as f:
            raw = f.read()
        assert '"a"' in raw
        assert '"b"' in raw

    def test_fields_with_spaces(self, tmp_path):
        path = str(tmp_path / "report.csv")
        save_csv_report(path, "hello world,foo bar")
        with open(path) as f:
            reader = csv.reader(f)
            rows = list(reader)
        assert rows[0] == ["hello world", "foo bar"]

    def test_overwrites_existing_file(self, tmp_path):
        path = str(tmp_path / "report.csv")
        save_csv_report(path, "first,pass")
        save_csv_report(path, "second,pass")
        with open(path) as f:
            reader = csv.reader(f)
            rows = list(reader)
        assert len(rows) == 1
        assert rows[0] == ["second", "pass"]

    def test_multiline_report_string_format(self, tmp_path):
        """Simulates the format from Habitat attribute manager reports."""
        report = "Handle,Type,Source\n/path/to/asset.glb,render,file\n/another.glb,collision,default"
        path = str(tmp_path / "report.csv")
        save_csv_report(path, report)
        with open(path) as f:
            reader = csv.reader(f)
            rows = list(reader)
        assert len(rows) == 3
        assert rows[0] == ["Handle", "Type", "Source"]
        assert rows[1][0] == "/path/to/asset.glb"

    def test_single_column(self, tmp_path):
        """Lines with no commas should produce single-field rows."""
        path = str(tmp_path / "report.csv")
        save_csv_report(path, "one\ntwo\nthree")
        with open(path) as f:
            reader = csv.reader(f)
            rows = list(reader)
        assert len(rows) == 3
        assert rows[0] == ["one"]
        assert rows[1] == ["two"]
        assert rows[2] == ["three"]
