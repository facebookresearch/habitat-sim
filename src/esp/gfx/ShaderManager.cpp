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

template <typename... ShaderArgs>
Shader::ptr ShaderManager::createShader(std::string id, ShaderArgs&&... args) {
  // insert a nullptr first to avoid useless construction when shader with id
  // already exists
  auto inserted = shaders_.emplace(std::move(id), nullptr);
  if (!inserted.second) {
    LOG(ERROR) << "Shader with id " << inserted.first->first
               << " already exists";
    return nullptr;
  }

  // now actually "insert" the shader
  inserted.first->second.reset(new Shader{std::forward<ShaderArgs>(args)...});
  LOG(INFO) << "Created Shader: " << inserted.first->first;
  return inserted.first->second;
}

template <typename... ShaderArgs>
Shader::ptr ShaderManager::getOrCreateShader(std::string id,
                                             ShaderArgs&&... args) {
  return getShader(id) ||
         createShader(std::move(id), std::forward<ShaderArgs>(args));
}

bool ShaderManager::deleteShader(const std::string& id) {
  return shaders_.erase(id);
}

}  // namespace gfx
}  // namespace esp
