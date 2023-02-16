# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import re
import sys


# TODO make this less brittle
sys.path = [
    os.path.join(os.path.dirname(__file__), "..", "src_python"),
    # os.path.join(os.path.dirname(__file__), '../build-bundledmagnum/src/deps/magnum-bindings/src/python/')
] + sys.path


import habitat_sim

# TODO: remove once m.css handles class hierarchies better
habitat_sim.logging.HabitatSimFormatter.formatStack.__doc__ = ""
# Monkey patch the registry to be the _Registry class instead of the singleton for docs
habitat_sim.registry = type(habitat_sim.registry)
# TODO: remove once utils/__init__.py is removed again
habitat_sim.utils.__all__.remove("quat_from_angle_axis")
habitat_sim.utils.__all__.remove("quat_rotate_vector")

PROJECT_TITLE = "Habitat"
PROJECT_SUBTITLE = "Sim Docs"
PROJECT_LOGO = "habitat.svg"
FAVICON = "habitat-blue.png"
MAIN_PROJECT_URL = "/"
INPUT_MODULES = [habitat_sim]
INPUT_DOCS = ["docs.rst", "gfx.rst", "noise_models.rst"]
INPUT_PAGES = [
    "pages/index.rst",
    "pages/new-actions.rst",
    "pages/attributesJSON.rst",
    "pages/stereo-agent.rst",
    "pages/lighting-setups.rst",
    "pages/image-extractor.rst",
    "pages/asset-viewer-tutorial.rst",
    "pages/managed-rigid-object-tutorial.rst",
    "pages/logging.rst",
    "pages/coordinate-frame-tutorial.rst",
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
    # I have no idea what is going on with this thing -- it reports itself as
    # being from the builtins module?
    "quaternion": "quaternion.quaternion",
    # TODO: remove once the inventory file contains this info
    "_magnum": "magnum",
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
            ("Attributes JSON", "attributesJSON"),
            ("Stereo agent", "stereo-agent"),
            ("Lighting Setups", "lighting-setups"),
            ("Image extraction", "image-extractor"),
            ("View Assets in Habitat-sim", "asset-viewer-tutorial"),
            ("Managed Rigid Object Tutorial", "managed-rigid-object-tutorial"),
            ("Coordinate Frame Tutorial", "coordinate-frame-tutorial"),
        ],
    ),
    ("Classes", "classes", []),
]
# When adding new pages / tutorials to LINKS_NAVBAR, the same option in
# Doxyfile-mcss needs to be updated accordingly to keep the C++ and Python
# navbar in sync.
LINKS_NAVBAR2 = [
    ("C++ API", "./cpp.html", []),
    ("Habitat Lab Docs", "../habitat-lab/index.html", []),
]

FINE_PRINT = f"""
| {PROJECT_TITLE} {PROJECT_SUBTITLE}. Copyright © 2021 Facebook AI Research.
| `Terms of Use </terms-of-use>`_ `Data Policy </data-policy>`_ `Cookie Policy </cookie-policy>`_
| Created with `m.css Python doc generator <https://mcss.mosra.cz/documentation/python/>`_."""
THEME_COLOR = "#478cc3"
STYLESHEETS = [
    "https://fonts.googleapis.com/css?family=Source+Sans+Pro:400,400i,600,600i%7CSource+Code+Pro:400,400i,600",
    "theme.compiled.css",
]

M_SPHINX_INVENTORIES = [
    ("python.inv", "https://docs.python.org/3/", [], ["m-doc-external"]),
    ("numpy.inv", "https://docs.scipy.org/doc/numpy/", [], ["m-doc-external"]),
    (
        "quaternion.inv",
        "https://quaternion.readthedocs.io/en/latest/",
        [],
        ["m-doc-external"],
    ),
    (
        "magnum-bindings.inv",
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

_hex_colors_src = re.compile(
    r"""<span class="s2">&quot;0x(?P<hex>[0-9a-f]{6})&quot;</span>"""
)
_hex_colors_dst = r"""<span class="s2">&quot;0x\g<hex>&quot;</span><span class="m-code-color" style="background-color: #\g<hex>;"></span>"""

M_CODE_FILTERS_POST = {
    ("Python", "string_hex_colors"): lambda code: _hex_colors_src.sub(
        _hex_colors_dst, code
    )
}

M_DOX_TAGFILES = [
    (
        "corrade.tag",
        "https://doc.magnum.graphics/corrade/",
        ["Corrade::"],
        ["m-doc-external"],
    ),
    (
        "magnum.tag",
        "https://doc.magnum.graphics/magnum/",
        ["Magnum::"],
        ["m-doc-external"],
    ),
    ("../build/docs/habitat-cpp.tag", "../habitat-sim/", [], ["m-doc-external"]),
]
