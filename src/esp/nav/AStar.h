// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/esp.h"
#include "esp/nav/ActionSpaceGraph.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace esp {
namespace nav {
namespace impl {
class MinHeap;
}

class AStar {
 public:
  struct Node {
    ESP_SMART_POINTERS(Node);
    uint64_t key;

    bool in_open = false, in_closed = false;

    int visit_order, steps, pos_in_heap;
    float cost = std::numeric_limits<float>::max(),
          h = std::numeric_limits<float>::max();
    ptr parent = nullptr;
    float h_weight = 1.0;

    inline bool operator==(const Node& o) const { return o.key == this->key; }

    inline float sort_weight(void) const {
      return this->cost + this->h_weight * this->h;
    }

    inline bool operator<(const Node& o) const {
      return std::make_pair(this->sort_weight(), this->visit_order) <
             std::make_pair(o.sort_weight(), o.visit_order);
    }
  };

  struct SearchState {
    std::unique_ptr<impl::MinHeap> open_set = std::make_unique<impl::MinHeap>();

    using MapType = std::unordered_map<uint64_t, Node::ptr>;
    std::unique_ptr<MapType> key_to_node = std::make_unique<MapType>();

    std::unordered_set<uint64_t> startKeys;

    impl::ActionSpaceGraph* graph = nullptr;
    int visit_order = 0;

    ESP_SMART_POINTERS(SearchState)
  };

  virtual bool compute(SearchState::ptr& ss,
                       const std::unordered_set<uint64_t>& exit_nodes,
                       std::vector<AStar::Node::ptr>& _path,
                       float heuristic_weight = 1.00);

  static SearchState::ptr CreateSearchState(
      impl::ActionSpaceGraph* graph,
      const std::vector<uint64_t>& starts);

 protected:
  virtual void update_vertex(SearchState::ptr& ss,
                             Node::ptr& s,
                             Node::ptr& sprime);
  virtual float compute_cost(SearchState::ptr& ss,
                             Node::ptr& s,
                             Node::ptr& sprime);
  virtual bool extract_path(SearchState::ptr& ss,
                            Node::ptr& start_node,
                            std::vector<Node::ptr>& _path);
};

namespace impl {

class MinHeap {
 public:
  inline int parent(int i) { return (i - 1) / 2; }
  inline int left(int i) { return (2 * i + 1); }
  inline int right(int i) { return (2 * i + 2); }

  AStar::Node::ptr pop(void);
  void push(AStar::Node::ptr& n);
  void decrease(AStar::Node::ptr& n);
  void heapify(AStar::Node::ptr& n);

  inline void swap(int i1, int i2) {
    auto tmp = heap[i1];
    heap[i1] = heap[i2];
    heap[i2] = tmp;

    heap[i1]->pos_in_heap = i1;
    heap[i2]->pos_in_heap = i2;
  }

  std::vector<AStar::Node::ptr> heap;

  inline void clear(void) { heap.clear(); }

  inline bool empty(void) const { return heap.empty(); }

  inline size_t size(void) const { return heap.size(); }

  ESP_SMART_POINTERS(MinHeap)
};
}  // namespace impl
}  // namespace nav
}  // namespace esp
