// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Check.h"

#include <Corrade/Utility/Assert.h>

namespace esp {
namespace core {

void (*throwInPython)(const char*) = nullptr;

/* [[noreturn]] will help the compiler optimize -- it basically tells it that
   the condition passed to HABITAT_EXCEPTION() can be assumed to be always true
   in the following code, because if not then the execution ends in this
   function. */
[[noreturn]] void throwIfInPythonOtherwiseAbort(const char* message) {
  /* The throwInPython function pointer gets filled during Python bindings
     startup. If it's nullptr, we're in plain C++ code. */
  if (throwInPython) {
    throwInPython(message);
    /* I failed to apply the NORETURN attribute to the throwInPython function
       pointer so at least this */
    CORRADE_INTERNAL_ASSERT_UNREACHABLE();
  }

  /* If it isn't defined, do an abort the same way as CORRADE_ASSERT(). This
     could be in an `else` block but I like to play with fire, so it's not --
     the NORETURN above should tell that to the compiler and the function
     should throw. */
  Corrade::Utility::Error{Corrade::Utility::Error::defaultOutput()} << message;
  std::abort();
}

}  // namespace core
}  // namespace esp
