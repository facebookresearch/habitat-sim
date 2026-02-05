# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# Inherit everything from the base config
import os, sys

sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from conf import *

# TODO: uncomment once m.sphinx can handle that
# OUTPUT = None

OUTPUT_STUBS = "../habitat_sim"
