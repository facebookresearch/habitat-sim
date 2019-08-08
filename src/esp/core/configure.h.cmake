// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#cmakedefine ESP_WITH_EGL
#cmakedefine BUILD_PTEX_SUPPORT

#ifdef CORRADE_TARGET_EMSCRIPTEN
#define USE_GLOG_SHIM
#endif
