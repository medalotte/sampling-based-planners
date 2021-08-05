/**
 *  MIT License
 *
 *  Copyright (c) 2019 Yuya Kudo
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <Node/SimpleNodeList/SimpleNodeList.h>

namespace planner {
SimpleNodeList::SimpleNodeList(const uint32_t &dim) : base::NodeListBase(dim), list_() {}

SimpleNodeList::~SimpleNodeList() {}

void SimpleNodeList::add(const NodePtr &node) {
  if (node->parent != nullptr) node->parent->is_leaf = false;
  list_.push_back(node);
}

void SimpleNodeList::init() { list_.clear(); }

int SimpleNodeList::getSize() { return list_.size(); }

SimpleNodeList::NodePtr SimpleNodeList::searchNN(const NodePtr &node) {
  NodePtr ret_node;
  auto min_dist = std::numeric_limits<double>::max();
  for (const auto &v : list_) {
    auto dist = v->state.distanceFrom(node->state);
    if (dist < min_dist) {
      ret_node = v;
      min_dist = dist;
    }
  }
  return ret_node;
}

std::vector<SimpleNodeList::NodePtr> SimpleNodeList::searchNBHD(const NodePtr &node, const double &radius) {
  std::vector<NodePtr> ret_nodes;
  for (const auto &v : list_) {
    auto dist = v->state.distanceFrom(node->state);
    if (dist < radius) {
      ret_nodes.push_back(v);
    }
  }
  return ret_nodes;
}

std::vector<SimpleNodeList::NodePtr> SimpleNodeList::searchLeafs() {
  std::vector<NodePtr> ret_nodes;
  for (const auto &v : list_) {
    if (v->is_leaf) {
      ret_nodes.push_back(v);
    }
  }
  return ret_nodes;
}
}  // namespace planner
