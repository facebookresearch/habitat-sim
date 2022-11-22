#pragma once


#include "esp/assets/Asset.h"
#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "esp/gfx/LightSetup.h"

// TODO:
#include <cstdint>
#include <functional>

namespace esp {
namespace gfx {
namespace replay {

typedef size_t GfxReplayNode;

/**
 * @brief Callbacks that contain the graphics replay implementation for gfx::replay::Player.
 */
struct PlayerCallbacks
{
    using LoadAndCreateRenderAssetInstance =
    std::function<GfxReplayNode*(
        const esp::assets::AssetInfo&,
        const esp::assets::RenderAssetInstanceCreationInfo&)>;

    using ChangeLightSetup =
        std::function<void(const gfx::LightSetup& lights)>;

    LoadAndCreateRenderAssetInstance loadAndCreateRenderInstance_ = {};
    ChangeLightSetup changeLightSetup_ = {};
};

}
}
}