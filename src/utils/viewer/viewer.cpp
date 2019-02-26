// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/gfx/Viewer.h"

int main(int argc, char** argv) {
  esp::gfx::Viewer app({argc, argv});
  return app.exec();
}
