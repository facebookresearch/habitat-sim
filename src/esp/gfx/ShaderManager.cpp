// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ShaderManager.h"

namespace esp {
namespace gfx {

Shader::ptr ShaderManager::getShader(const std::string& id) {
  auto it = shaders_.find(id);
  return it == shaders_.end() ? nullptr : it->second;
}

Shader::cptr ShaderManager::getShader(const std::string& id) const {
  auto it = shaders_.find(id);
  return it == shaders_.end() ? nullptr : it->second;
}

bool ShaderManager::deleteShader(const std::string& id) {
  return shaders_.erase(id);
}

}  // namespace gfx
}  // namespace esp
