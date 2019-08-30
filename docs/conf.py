# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import sys

# TODO make this less brittle
sys.path = [
    os.path.join(os.path.dirname(__file__), "../"),
    # os.path.join(os.path.dirname(__file__), '../build-bundledmagnum/src/deps/magnum-bindings/src/python/')
] + sys.path

import habitat_sim  # NOQA

# TODO: remove once m.css handles class hierarchies better
habitat_sim.logging.GlogFormatter.formatStack.__doc__ = ""

PROJECT_TITLE = "Habitat"
PROJECT_SUBTITLE = "Sim Python docs"
MAIN_PROJECT_URL = "https://aihabitat.org"
INPUT_MODULES = [habitat_sim]
INPUT_DOCS = ["docs.rst"]
INPUT_PAGES = [
    "pages/index.rst",
    "pages/new-actions.rst",
    "pages/stereo-agent.rst",
    "pages/notebooks.rst",
]

PLUGINS = [
    "m.abbr",
    "m.code",
    "m.components",
    "m.dox",
    "m.gh",
    "m.htmlsanity",
    "m.images",
    "m.link",
    "m.math",
    "m.sphinx",
]

CLASS_INDEX_EXPAND_LEVELS = 2

NAME_MAPPING = {
    # TODO: remove once the inventory file contains this info
    "_magnum": "magnum"
}
PYBIND11_COMPATIBILITY = True
ATTRS_COMPATIBILITY = True

OUTPUT = "../build/docs/habitat-sim/"

LINKS_NAVBAR1 = [
    (
        "Pages",
        "pages",
        [
            ("Add new actions", "new-actions"),
            ("Stereo agent", "stereo-agent"),
            ("Notebooks", "notebooks"),
        ],
    ),
    ("Classes", "classes", []),
]
LINKS_NAVBAR2 = [
    ("C++ Docs", "../habitat-cpp/index.html", []),
    ("Habitat API", "https://aihabitat.org/habitat-api/", []),
]

M_SPHINX_INVENTORIES = [
    # TODO: clean up once paths are relative to this file implicitly
    (
        os.path.join(os.path.realpath(os.path.dirname(__file__)), "python.inv"),
        "https://docs.python.org/3/",
        [],
        ["m-doc-external"],
    ),
    (
        os.path.join(os.path.realpath(os.path.dirname(__file__)), "numpy.inv"),
        "https://docs.scipy.org/doc/numpy/",
        [],
        ["m-doc-external"],
    ),
    (
        os.path.join(
            os.path.realpath(os.path.dirname(__file__)), "magnum-bindings.inv"
        ),
        "https://doc.magnum.graphics/python/",
        [],
        ["m-doc-external"],
    ),
]
M_SPHINX_INVENTORY_OUTPUT = "objects.inv"
M_SPHINX_PARSE_DOCSTRINGS = True

M_HTMLSANITY_SMART_QUOTES = True
# Will people hate me if I enable this?
# M_HTMLSANITY_HYPHENATION = True
