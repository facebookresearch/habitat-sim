# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

find_package(Magnum REQUIRED Trade)

# If Magnum is built as static (i.e., the bundled submodule), the plugin is
# static as well, to avoid unnecessary pain with duplicated symbols. With
# dynamic (and thus probably external) Magnum it's dynamic so it can be loaded
# by external tools such as magnum-player as well.
if(MAGNUM_BUILD_STATIC)
  corrade_add_static_plugin(HabitatImporter ${CMAKE_CURRENT_BINARY_DIR} "" HabitatImporter.conf HabitatImporter.cpp)
else()
  corrade_add_plugin(HabitatImporter ${CMAKE_CURRENT_BINARY_DIR} "" HabitatImporter.conf HabitatImporter.cpp)
endif()
target_link_libraries(HabitatImporter PUBLIC Magnum::Trade)
