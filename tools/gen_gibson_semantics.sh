#!/bin/bash

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

# gen_gibson_semantics.sh - generate semantic scene from .npz data from
#                           https://3dscenegraph.stanford.edu/

NPZ_PATH=$1
OBJ_PATH=$2
OUT_PATH=$3

TOOLS_DIR=$(dirname "$0")

if [ $# -ne 3 ]; then
  echo "Usage: $0 NPZ_PATH OBJ_PATH OUT_PATH"
  exit 1
fi

for npz in "${NPZ_PATH}"/*.npz; do
  filename=$(basename "${npz}")
  tmp=${filename#3DSceneGraph_}
  scene=${tmp%.npz}
  echo "${scene}"
  "${TOOLS_DIR}"/npz2ids.py "${npz}" "${OUT_PATH}"/"${scene}".ids
  "${TOOLS_DIR}"/npz2scn.py "${npz}" "${OUT_PATH}"/"${scene}".scn
  "${TOOLS_DIR}"/../build/utils/datatool/datatool create_gibson_semantic_mesh "${OBJ_PATH}"/"${scene}"/mesh.obj "${OUT_PATH}"/"${scene}".ids "${OUT_PATH}"/"${scene}"_semantic.ply
done
