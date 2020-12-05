// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief Include file for all helper functions for serializing types to
 * rapidjson.
 *
 * These helpers are designed so that every builtin type and user type can be
 * serialized/deserialized with esp::io::AddMember/ReadMember. The leads to
 * uniform and readable serialization code. To achieve this, every complex user
 * type defines ToRJsonValue/FromRJsonValue, and then template versions of
 * AddMember/ReadMember will automatically use these.
 *
 * See IOTest.cpp for example usage.
 */

#include "JsonBuiltinTypes.h"

// ToRJsonValue/FromRJsonValue for all user types should go in the headers
// below.
#include "JsonEspTypes.h"
#include "JsonMagnumTypes.h"

// This must go after all ToRJsonValue/FromRJsonValue. The quirky ordering is to
// avoid some compile errors.
#include "JsonBuiltinTypes.hpp"
