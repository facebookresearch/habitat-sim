// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_MATHINCLUDE_H_
#define ESP_CORE_MATHINCLUDE_H_

// The _USE_MATH_DEFINES macro is used to enable the definition of mathematical
// constants like M_PI and M_PI_2 in the <cmath> header. This is typically
// needed in Microsoft Visual Studio environments where these constants are not
// defined by default.
#define _USE_MATH_DEFINES
#include <cmath>

// In environments other than Microsoft Visual Studio, or when <cmath> is
// included before this header by external dependencies, the _USE_MATH_DEFINES
// macro may not enable the definition of M_PI and M_PI_2. In such cases, we
// provide manual definitions for these constants to ensure they are available.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923
#endif

#endif  // ESP_CORE_MATHINCLUDE_H_
