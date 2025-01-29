// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RenderInstanceHelper.h"
#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "Simulator.h"
#include "esp/core/Esp.h"

#include <cassert>

namespace esp {
namespace sim {

RenderInstanceHelper::RenderInstanceHelper(Simulator& sim, const std::vector<float>& identityRotation) {

    sim_ = &sim;
    CORRADE_INTERNAL_ASSERT(identityRotation[0] == 1.f || identityRotation[3] == 1.f);
    isXYZW_ = identityRotation[3] == 1.f;
}

int RenderInstanceHelper::AddInstance(const std::string& assetFilepath, int semanticId) {

    esp::assets::AssetInfo assetInfo;
    assetInfo.filepath = assetFilepath;
    assetInfo.forceFlatShading = false;

    esp::assets::RenderAssetInstanceCreationInfo::Flags flags;
    // flags |= RenderAssetInstanceCreationInfo::Flag::IsStatic;
    flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsRGBD;
    flags |= esp::assets::RenderAssetInstanceCreationInfo::Flag::IsSemantic;
    const Corrade::Containers::Optional<Magnum::Vector3> scale = Cr::Containers::NullOpt;

    assets::RenderAssetInstanceCreationInfo creation(
        assetFilepath, scale, flags, DEFAULT_LIGHTING_KEY);

    auto* node = sim_->loadAndCreateRenderAssetInstance(assetInfo, creation);
    instances_.push_back(node);

    setSemanticIdForSubtree(node, semanticId);

    return instances_.size() - 1;
}

int RenderInstanceHelper::GetNumInstances() {
    return instances_.size();
}

void RenderInstanceHelper::ClearAllInstances() {

    for (auto* node : instances_) {
        delete node;
    }
    instances_.clear();
}

void RenderInstanceHelper::SetWorldPoses(float* positions, size_t positionsSize, float* orientations, size_t orientationsSize) {

    CORRADE_INTERNAL_ASSERT(positionsSize == GetNumInstances() * 3);
    CORRADE_INTERNAL_ASSERT(orientationsSize == GetNumInstances() * 4);
    float* p = positions;
    float* o = orientations;
    for (auto* node : instances_) {

        node->setTranslation(Magnum::Vector3(p[0], p[1], p[2]));
        p += 3;
        Mn::Quaternion rotation;
        rotation = isXYZW_ 
            ? Mn::Quaternion(Magnum::Vector3(o[0], o[1], o[2]), o[3])
            : Mn::Quaternion(Magnum::Vector3(o[1], o[2], o[3]), o[0]);
        o += 4;
        // note: avoid CORRADE_INTERNAL_ASSERT because it doesn't compile out in optimized builds
        // assert(rotation.isNormalized());
        node->setRotation(rotation);
    }
}


}  // namespace sim
}  // namespace esp
