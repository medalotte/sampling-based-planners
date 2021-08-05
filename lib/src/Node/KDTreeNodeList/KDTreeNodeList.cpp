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

#include <Node/KDTreeNodeList/KDTreeNodeList.h>

namespace planner {
KDTreeNodeList::KDTreeNodeList(const uint32_t &dim) : base::NodeListBase(dim), root_(nullptr), depth_(0) {}

KDTreeNodeList::~KDTreeNodeList() {}

void KDTreeNodeList::add(const NodePtr &node) {
  if (node->parent != nullptr) node->parent->is_leaf = false;
  nodes_.push_back(node);

  if (std::log2(nodes_.size()) * (1 / REBALANCE_RATIO) <= depth_) {
    // rebuild balanced kd-tree
    clearRec(root_);
    root_ = nullptr;
    depth_ = 0;
    std::vector<int> indices(nodes_.size());
    std::iota(indices.begin(), indices.end(), 0);
    root_ = buildRec(indices, 0, (int)nodes_.size(), 0);
  } else {
    // insert the node to kd-tree
    insertRec(root_, nodes_.size() - 1, 0);
  }
}

void KDTreeNodeList::init() {
  clearRec(root_);
  root_ = nullptr;
  depth_ = 0;
  nodes_.clear();
}

int KDTreeNodeList::getSize() { return nodes_.size(); }

KDTreeNodeList::NodePtr KDTreeNodeList::searchNN(const NodePtr &node) {
  NodePtr ret_node;
  auto min_dist = std::numeric_limits<double>::max();
  searchNNRec(node, root_, ret_node, min_dist);
  return ret_node;
}

std::vector<KDTreeNodeList::NodePtr> KDTreeNodeList::searchNBHD(const NodePtr &node, const double &radius) {
  std::vector<NodePtr> ret_nodes;
  searchNBHDRec(node, root_, ret_nodes, radius);
  return ret_nodes;
}

std::vector<KDTreeNodeList::NodePtr> KDTreeNodeList::searchLeafs() {
  std::vector<NodePtr> ret_nodes;
  for (const auto &v : nodes_) {
    if (v->is_leaf) {
      ret_nodes.push_back(v);
    }
  }
  return ret_nodes;
}

void KDTreeNodeList::clearRec(const KDNodePtr &node) {
  if (node == nullptr) return;
  if (node->child_r) {
    clearRec(node->child_r);
    node->child_r = nullptr;
  }
  if (node->child_l) {
    clearRec(node->child_l);
    node->child_l = nullptr;
  }
}

KDTreeNodeList::KDNodePtr KDTreeNodeList::buildRec(std::vector<int> &indices, const int &offset, const int &npoints,
                                                   const int &depth) {
  if (npoints <= 0) {
    return nullptr;
  }
  depth_ = std::max(depth_, depth);

  const int axis = depth % DIM;
  const int mid = (npoints - 1) / 2;
  auto comp = [&](const int &lhs, const int &rhs) {
    return nodes_[lhs]->state.vals[axis] < nodes_[rhs]->state.vals[axis];
  };
  std::nth_element(indices.begin() + offset, indices.begin() + offset + mid, indices.begin() + offset + npoints, comp);

  auto node = std::make_shared<KDTreeNode>();
  node->idx = indices[offset + mid];
  node->axis = axis;
  node->child_r = buildRec(indices, offset, mid, depth + 1);
  node->child_l = buildRec(indices, offset + mid + 1, npoints - mid - 1, depth + 1);
  return node;
}

KDTreeNodeList::KDNodePtr KDTreeNodeList::insertRec(const KDNodePtr &root, const int &new_node_index,
                                                    const int &depth) {
  auto axis = depth % DIM;
  if (root == nullptr) {
    auto node = std::make_shared<KDTreeNode>();
    node->idx = new_node_index;
    node->axis = axis;

    depth_ = std::max(depth_, depth);
    if (depth_ == 0) {
      root_ = node;
    }
    return node;
  } else {
    if (nodes_[new_node_index]->state.vals[axis] < (nodes_[root->idx]->state.vals[axis])) {
      root->child_r = insertRec(root->child_r, new_node_index, depth + 1);
    } else {
      root->child_l = insertRec(root->child_l, new_node_index, depth + 1);
    }
    return root;
  }
}

void KDTreeNodeList::searchNNRec(const NodePtr &query, const KDNodePtr node, NodePtr &guess, double &min_dist) const {
  if (node == nullptr) {
    return;
  }

  const NodePtr &train = nodes_[node->idx];
  const double dist = query->state.distanceFrom(train->state);
  if (dist < min_dist) {
    min_dist = dist;
    guess = nodes_[node->idx];
  }

  const int axis = node->axis;
  const int dir = query->state.vals[axis] < train->state.vals[axis] ? 0 : 1;
  searchNNRec(query, dir == 0 ? node->child_r : node->child_l, guess, min_dist);

  const double diff = fabs(query->state.vals[axis] - train->state.vals[axis]);
  if (diff < min_dist) {
    searchNNRec(query, dir == 0 ? node->child_l : node->child_r, guess, min_dist);
  }
}

void KDTreeNodeList::searchNBHDRec(const NodePtr &query, const KDNodePtr node, std::vector<NodePtr> &near_nodes,
                                   const double &radius) const {
  if (node == nullptr) {
    return;
  }

  const NodePtr &train = nodes_[node->idx];
  const double dist = query->state.distanceFrom(train->state);
  if (dist < radius) near_nodes.push_back(nodes_[node->idx]);

  const int axis = node->axis;
  const int dir = query->state.vals[axis] < train->state.vals[axis] ? 0 : 1;
  searchNBHDRec(query, dir == 0 ? node->child_r : node->child_l, near_nodes, radius);

  const double diff = fabs(query->state.vals[axis] - train->state.vals[axis]);
  if (diff < radius) {
    searchNBHDRec(query, dir == 0 ? node->child_l : node->child_r, near_nodes, radius);
  }
}
}  // namespace planner
