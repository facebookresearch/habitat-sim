#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import ctypes

# must call this before importing habitat or magnum! avoids EGL_BAD_ACCESS error on some platforms
import sys

flags = sys.getdlopenflags()
sys.setdlopenflags(flags | ctypes.RTLD_GLOBAL)

import magnum as mn
from magnum.platform.glfw import Application


class MyApplication(Application):
    def __init__(self, args):
        super().__init__(args)
        self.anim_fraction = 0

    def draw_event(self):
        self.anim_fraction = (self.anim_fraction + 0.05) % 1

        mn.gl.Renderer.clear_color = mn.Color4(self.anim_fraction, 0, 0, 1)
        mn.gl.default_framebuffer.clear(mn.gl.FramebufferClear.COLOR)
        self.swap_buffers()
        self.redraw()


if __name__ == "__main__":
    glfw_config = Application.Configuration()
    glfw_config.title = "Sandbox App"
    glfw_config.size = (400, 300)
    app = MyApplication(glfw_config)
    app.exec()
