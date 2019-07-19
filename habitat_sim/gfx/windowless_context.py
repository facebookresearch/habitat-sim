#!/usr/bin/env python3

# Copyright (c) Facebook, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import habitat_sim.bindings as hsim
from habitat_sim.utils import Singleton


class _WindowlessContextSingleton(metaclass=Singleton):
    def __init__(self):
        self._ctx = None

    def get(self, gpu_id):
        if self._ctx is None:
            self._ctx = hsim.WindowlessContext(gpu_id)
        else:
            assert (
                gpu_id == self._ctx.gpu_device_id
            ), "Cannot change the GPU of the context"

        return self._ctx


WindowlessContextSingleton = _WindowlessContextSingleton()
