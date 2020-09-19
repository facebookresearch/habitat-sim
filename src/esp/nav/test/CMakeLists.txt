# (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

find_package(Corrade REQUIRED Utility TestSuite)

configure_file(configure.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/configure.h)

corrade_add_test(PathFinderTest PathFinderTest.cpp LIBRARIES nav Corrade::Utility)
target_include_directories(PathFinderTest PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
