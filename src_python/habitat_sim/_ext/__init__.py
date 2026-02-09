# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# This file makes _ext a proper Python package so that the compiled
# C++ extension module (habitat_sim_bindings.so) can be imported as
# habitat_sim._ext.habitat_sim_bindings.
#
# The actual .so file is:
#   - Placed here by CMake during non-editable installs (pip install .)
#   - Resolved by scikit-build-core's redirecting finder during editable
#     installs (pip install -e .)
