#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# import magnum as mn
from magnum.platform.glfw import Application


class MyApplication(Application):
    def draw_event(self):
        pass


if __name__ == "__main__":
    glfw_config = Application.Configuration()
    glfw_config.title = "Sandbox App"
    glfw_config.size = (400, 300)
    app = MyApplication(glfw_config)
    app.exec()
