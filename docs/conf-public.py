# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Inherit everything from the local config
import os, sys

sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from conf import *

OUTPUT = "../build/docs-public/habitat-sim/"

SEARCH_DOWNLOAD_BINARY = "searchdata-python-v1.bin"
SEARCH_BASE_URL = "https://aihabitat.org/docs/habitat-sim/"
SEARCH_EXTERNAL_URL = "https://google.com/search?q=site:aihabitat.org+{query}"

assert M_DOX_TAGFILES[2][0] == "../build/docs/habitat-cpp.tag"
M_DOX_TAGFILES[2] = (
    "../build/docs-public/habitat-cpp.tag",
    "../habitat-sim/",
    [],
    ["m-doc-external"],
)
