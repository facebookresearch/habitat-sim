// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_MAGNUM_H_
#define ESP_GFX_MAGNUM_H_

#include <Magnum/SceneGraph/SceneGraph.h>

typedef Magnum::SceneGraph::Object<
    Magnum::SceneGraph::TranslationRotationScalingTransformation3D>
    MagnumObject;
typedef Magnum::SceneGraph::Scene<
    Magnum::SceneGraph::TranslationRotationScalingTransformation3D>
    MagnumScene;
typedef Magnum::SceneGraph::Camera3D MagnumCamera;
typedef Magnum::SceneGraph::Drawable3D MagnumDrawable;
typedef Magnum::SceneGraph::DrawableGroup3D MagnumDrawableGroup;

#endif  // ESP_GFX_MAGNUM_H_
