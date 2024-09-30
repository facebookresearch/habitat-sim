// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AbstractAttributes.h"
#include "esp/physics/PhysicsObjectBase.h"
namespace esp {

namespace metadata {
namespace attributes {

AbstractAttributes::AbstractAttributes(const std::string& attributesClassKey,
                                       const std::string& handle)
    : Configuration() {
  // set up an existing subgroup for user_defined attributes
  addOrEditSubgroup<Configuration>("user_defined");

  init("handle", handle);
  // These fields are all hidden/internally used fields, so do not set them as
  // initialized/default values. They still should never be written to file.
  setHidden("__attributesClassKey", attributesClassKey);
  setHidden("__ID", 0);
  setHidden("__fileDirectory", "");
  setHidden("__actualFilename", "");
  // Initialize attributes to be different than on version on disk, if one
  // exists. This should be set to true on file load and on file save.
  setHidden("__isAttrSaved", false);
}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
