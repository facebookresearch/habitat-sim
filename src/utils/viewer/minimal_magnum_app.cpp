// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Magnum/Platform/GlfwApplication.h>

class MyApplication : public Magnum::Platform::Application {
 public:
  explicit MyApplication(const Arguments& arguments)
      : Magnum::Platform::Application{
            arguments,
            Configuration{}.setTitle("MyApplication").setSize({400, 300})} {}

  void drawEvent() override {
    printf("drawEvent\n");
    redraw();
  }
};

MAGNUM_APPLICATION_MAIN(MyApplication)
