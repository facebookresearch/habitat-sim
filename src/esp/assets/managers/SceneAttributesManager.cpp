// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneAttributesManager.h"
#include "AttributesManagerBase.h"

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/ConfigurationGroup.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>

namespace esp {
namespace assets {

namespace managers {

PhysicsSceneAttributes::ptr SceneAttributesManager::createAttributesTemplate(
    const std::string& sceneAttributesHandle,
    bool registerTemplate) {
  PhysicsSceneAttributes::ptr sceneAttributesTemplate =
      PhysicsSceneAttributes::create(sceneAttributesHandle);

  // TODO: Support scene file existence validation?  Does default scene
  // attributes have use/meaning?

  sceneAttributesTemplate->setRenderAssetHandle(sceneAttributesHandle);
  sceneAttributesTemplate->setCollisionAssetHandle(sceneAttributesHandle);

  if (registerTemplate) {
    registerAttributesTemplate(sceneAttributesTemplate, sceneAttributesHandle);
  }

  return sceneAttributesTemplate;
}  // SceneAttributesManager::createAttributesTemplate

// template class AttributesManager<PhysicsSceneAttributes>;

}  // namespace managers
}  // namespace assets
}  // namespace esp
