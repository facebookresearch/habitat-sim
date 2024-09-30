#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


from typing import Any, Dict

import magnum as mn
import numpy as np

import habitat_sim


# Class to display semantic settings in a scene
class SemanticDisplay:
    def __init__(self, sim: habitat_sim.simulator.Simulator):
        self.sim = sim
        # Descriptive strings for semantic region debug draw possible choices
        self.semantic_region_debug_draw_choices = ["None", "Kitchen Only", "All"]
        # draw semantic region debug visualizations if present : should be [0 : len(semantic_region_debug_draw_choices)-1]
        self.semantic_region_debug_draw_state = 0
        # Colors to use for each region's semantic rendering.
        self.debug_semantic_colors: Dict[str, mn.Color4] = {}

    def cycle_semantic_region_draw(self):
        new_state_idx = (self.semantic_region_debug_draw_state + 1) % len(
            self.semantic_region_debug_draw_choices
        )
        info_str = f"Change Region Draw from {self.semantic_region_debug_draw_choices[self.semantic_region_debug_draw_state]} to {self.semantic_region_debug_draw_choices[new_state_idx]}"

        # Increment visualize semantic bboxes. Currently only regions supported
        self.semantic_region_debug_draw_state = new_state_idx
        return info_str

    def draw_region_debug(self, debug_line_render: Any) -> None:
        """
        Draw the semantic region wireframes.
        """
        if self.semantic_region_debug_draw_state == 0:
            return
        if len(self.debug_semantic_colors) != len(self.sim.semantic_scene.regions):
            self.debug_semantic_colors = {}
            for region in self.sim.semantic_scene.regions:
                self.debug_semantic_colors[region.id] = mn.Color4(
                    mn.Vector3(np.random.random(3))
                )
        if self.semantic_region_debug_draw_state == 1:
            for region in self.sim.semantic_scene.regions:
                if "kitchen" not in region.id.lower():
                    continue
                color = self.debug_semantic_colors.get(region.id, mn.Color4.magenta())
                for edge in region.volume_edges:
                    debug_line_render.draw_transformed_line(
                        edge[0],
                        edge[1],
                        color,
                    )
        else:
            # Draw all
            for region in self.sim.semantic_scene.regions:
                color = self.debug_semantic_colors.get(region.id, mn.Color4.magenta())
                for edge in region.volume_edges:
                    debug_line_render.draw_transformed_line(
                        edge[0],
                        edge[1],
                        color,
                    )
