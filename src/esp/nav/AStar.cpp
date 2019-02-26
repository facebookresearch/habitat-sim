// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/nav/AStar.h"

namespace esp {
namespace nav {

AStar::SearchState::ptr AStar::CreateSearchState(
    impl::ActionSpaceGraph* graph,
    const std::vector<uint64_t>& starts) {
  auto ss = std::make_shared<SearchState>();
  ss->graph = graph;

  for (auto& key : starts) {
    auto n = std::make_shared<Node>();
    n->key = key;
    ss->key_to_node->emplace(n->key, n);

    n->cost = 0.0;
    n->visit_order = ss->visit_order++;

    ss->open_set->push(n);
    ss->startKeys.emplace(key);
  }

  return ss;
}

bool AStar::compute(SearchState::ptr& ss,
                    const std::unordered_set<uint64_t>& exit_nodes,
                    std::vector<AStar::Node::ptr>& _path,
                    float heuristic_weight) {
  _path.clear();
  if (ss->key_to_node->size() > 0) {
    for (auto& nk : exit_nodes) {
      auto it = ss->key_to_node->find(nk);
      if (it != ss->key_to_node->end() && it->second->in_closed) {
        return this->extract_path(ss, it->second, _path);
      }
    }
  }

  {
    auto new_open_set = new impl::MinHeap();
    new_open_set->heap.reserve(ss->open_set->size());
    for (auto& n : ss->open_set->heap) {
      n->h = ss->graph->heuristic(n->key);
      n->h_weight = heuristic_weight;
      new_open_set->push(n);
    }
    ss->open_set.reset(new_open_set);
  }

  while (!ss->open_set->empty()) {
    auto s = ss->open_set->pop();

    s->in_open = false;
    s->in_closed = true;

    for (const uint64_t& neighbor : ss->graph->edgesFrom(s->key)) {
      Node::ptr sprime;
      {
        auto it = ss->key_to_node->find(neighbor);
        if (it == ss->key_to_node->end()) {
          sprime = Node::ptr(new Node);
          sprime->key = neighbor;
          sprime->h = ss->graph->heuristic(sprime->key);
          sprime->h_weight = heuristic_weight;

          ss->key_to_node->emplace(sprime->key, sprime);
        } else {
          sprime = it->second;
        }
      }

      if (!sprime->in_closed) {
        this->update_vertex(ss, s, sprime);
      }
    }

    // This check MUST happen after the neighbors have been expanded
    // for the ability quickly search for a different goal to work
    if (exit_nodes.find(s->key) != exit_nodes.end()) {
      return this->extract_path(ss, s, _path);
    }
  }

  return false;
}

void AStar::update_vertex(SearchState::ptr& ss,
                          Node::ptr& s,
                          Node::ptr& sprime) {
  const float new_cost = this->compute_cost(ss, s, sprime);

  if (new_cost < sprime->cost) {
    sprime->cost = new_cost;
    sprime->parent = s;
    sprime->visit_order = ss->visit_order++;
    sprime->steps = s->steps + 1;

    if (sprime->in_open) {
      ss->open_set->decrease(sprime);
    } else {
      ss->open_set->push(sprime);
      sprime->in_open = true;
    }
  }
}

float AStar::compute_cost(SearchState::ptr& ss,
                          Node::ptr& s,
                          Node::ptr& sprime) {
  return s->cost + ss->graph->edgeWeight(s->key, sprime->key);
}

bool AStar::extract_path(SearchState::ptr& ss,
                         Node::ptr& start_node,
                         std::vector<Node::ptr>& _path) {
  Node::ptr n = start_node;
  while (n != nullptr) {
    _path.emplace_back(n);
    n = n->parent;
  }

  std::reverse(_path.begin(), _path.end());

  return (ss->startKeys.find(_path[0]->key) != ss->startKeys.end());
}

namespace impl {

AStar::Node::ptr MinHeap::pop(void) {
  swap(0, heap.size() - 1);
  auto root = heap.back();
  heap.pop_back();

  heapify(heap[0]);
  return root;
}

void MinHeap::heapify(AStar::Node::ptr& n) {
  const int i = n->pos_in_heap;
  const int l = left(i);
  const int r = right(i);
  int smallest = i;
  const int heap_size = static_cast<int>(heap.size());
  if (l < heap_size && (*heap[l]) < (*heap[smallest])) {
    smallest = l;
  }
  if (r < heap_size && (*heap[r]) < (*heap[smallest])) {
    smallest = r;
  }

  if (smallest != i) {
    swap(i, smallest);
    heapify(heap[smallest]);
  }
}

void MinHeap::push(AStar::Node::ptr& n) {
  heap.push_back(n);
  n->pos_in_heap = heap.size() - 1;
  decrease(n);
}

void MinHeap::decrease(AStar::Node::ptr& n) {
  int i = n->pos_in_heap;
  while (i != 0 && (*heap[i]) < (*heap[parent(i)])) {
    swap(i, parent(i));
    i = parent(i);
  }
}
}  // namespace impl
}  // namespace nav
}  // namespace esp
