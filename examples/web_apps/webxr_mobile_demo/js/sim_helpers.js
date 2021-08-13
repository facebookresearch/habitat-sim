// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

/* global Module */

export class SimHelpers {
  constructor(sim) {
    this.sim = sim;
  }

  // Hide an object by translating it far away.
  hide(objectId) {
    this.sim.setTranslation(new Module.Vector3(1000, 1000, 1000), objectId, 0);
  }

  exclusiveRaycast(ray, dist, excludes) {
    let raycastResults = this.sim.castRay(ray, dist, 0);
    let hits = raycastResults.hits;
    for (let i = 0; i < hits.size(); i++) {
      let hit = hits.get(i);
      if (!excludes.includes(hit.objectId)) {
        return hit;
      }
    }
    return null;
  }
}
