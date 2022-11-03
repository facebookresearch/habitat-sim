#!/bin/sh
#Synchronizes notebokos with script representations

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

pre-commit run 'jupytext' --files "$@"
