// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_MAGNUM_H_
#define ESP_GFX_MAGNUM_H_

#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/Mesh.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/TranslationRotationScalingTransformation3D.h>
#include <Magnum/configure.h>
// #include <Magnum/SceneGraph/MatrixTransformation3D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Trade/PhongMaterialData.h>

typedef Magnum::GL::Mesh MagnumMesh;
typedef Magnum::GL::Texture2D MagnumTexture2D;
typedef Magnum::SceneGraph::Object<
    Magnum::SceneGraph::TranslationRotationScalingTransformation3D>
    MagnumObject;
typedef Magnum::SceneGraph::Scene<
    Magnum::SceneGraph::TranslationRotationScalingTransformation3D>
    MagnumScene;
typedef Magnum::SceneGraph::Camera3D MagnumCamera;
typedef Magnum::SceneGraph::Drawable3D MagnumDrawable;
typedef Magnum::SceneGraph::DrawableGroup3D MagnumDrawableGroup;
typedef Magnum::GL::AbstractShaderProgram MagnumShaderProgram;
typedef Magnum::Trade::PhongMaterialData MagnumMaterialData;

#endif  // ESP_GFX_MAGNUM_H_
