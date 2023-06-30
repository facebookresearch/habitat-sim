// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Check.h"

#include <Corrade/Utility/Assert.h>

namespace esp {
namespace core {

void (*throwRuntimeInPython)(const char*) = nullptr;

/* [[noreturn]] will help the compiler optimize -- it basically tells it that
   the condition passed to HABITAT_EXCEPTION() can be assumed to be always true
   in the following code, because if not then the execution ends in this
   function. */

[[noreturn]] void throwIfInPythonOtherwiseExit(const char* message) {
  /* The throwRuntimeInPython function pointer gets filled during Python
     bindings startup. If it's nullptr, we're in plain C++ code. */
  if (throwRuntimeInPython) {
    throwRuntimeInPython(message);

    std::exit(1);
  }

  /*
   * If it isn't defined, display a Fatal message, which will terminate with
   * std::exit(1).
   */
  Corrade::Utility::Fatal{Corrade::Utility::Fatal::defaultOutput(), 1}
      << message;
}

}  // namespace core
}  // namespace esp
