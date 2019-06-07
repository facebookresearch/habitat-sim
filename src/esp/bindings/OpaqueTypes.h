// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <map>
#include <vector>

#include "esp/core/esp.h"
#include "esp/nav/GreedyFollower.h"

// Support for types with commas is implemented since
// https://github.com/pybind/pybind11/commit/e88656ab45ae75df7dcb1fcdd2c89805b52e4665,
// which is not in any released version yet. Use a typedef until then.
typedef std::map<std::string, std::string> map_string_string;
PYBIND11_MAKE_OPAQUE(map_string_string);
typedef std::vector<esp::nav::GreedyGeodesicFollowerImpl::CODES> vector_codes;
PYBIND11_MAKE_OPAQUE(vector_codes);
