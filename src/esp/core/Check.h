// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_CHECK_H_
#define ESP_CORE_CHECK_H_

#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/Macros.h>
#include <sstream>

/** @file
  @brief ESP_CHECK macro, for use with fatal runtime errors.

  Below is an overview of asserts, ESP_CHECK, exception-throwing, and warnings
  in Habitat-sim. These are new guidelines as of Feb 2021; not all Habitat code
  follows these guidelines yet.

  assert
  - see CORRADE_ASSERT and CORRADE_ASSERT_INTERNAL.
  - A failed assert represents a Hab programmer error.
  - It shouldn't depend on Hab-user-provided input or data.
  - If/when we temporarily disable asserts (e.g. as an optimization), the Hab
  library should still work correctly.

  runtime error
  - See fatal and recoverable variants below.
  - Something outside the Hab programmer's control went wrong.
  - E.g. bad user input or data, bad OS behavior.

  fatal (non-recoverable) runtime error
  - see ESP_CHECK.
  - We consider the error unrecoverable, because it's impossible or inconvenient
  to handle the error in a recoverable way.
  - We terminate the program (including the Python user program calling into
  Hab).

  recoverable (non-fatal) runtime error
  - See throw std::runtime_error in GfxBindings.cpp and other binding code.
  - By recover, we mean throw a Python exception or otherwise abort the
  user-requested operation but still allow the program to continue.
  - In general, we only throw exceptions in binding code, not elsewhere in the
  C++.

  warning
  - see ESP_WARNING()
  - A message to the user telling her she *probably* did something wrong (e.g.
  bad input, bad data)
*/

namespace esp {
namespace core {

/* The throwInPython function pointer gets filled during Python bindings
   startup. If it's nullptr, we're in plain C++ code. */
extern void (*throwInPython)(const char*);

// For use in ESP_CHECK
[[noreturn]] void throwIfInPythonOtherwiseAbort(const char* message);

}  // namespace core
}  // namespace esp

/* A runtime check that must pass, otherwise we consider this a fatal runtime
error. The program terminates with the supplied error message. */
#define ESP_CHECK(condition, ...)                                 \
  do {                                                            \
    if (!(condition)) {                                           \
      std::ostringstream out;                                     \
      Corrade::Utility::Debug{                                    \
          &out, Corrade::Utility::Debug::Flag::NoNewlineAtTheEnd} \
          << "ESP_CHECK failed:" << __VA_ARGS__;                  \
      esp::core::throwIfInPythonOtherwiseAbort(out.str().data()); \
    }                                                             \
  } while (false)

#endif
