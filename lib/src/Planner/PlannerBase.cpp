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

#include <Planner/PlannerBase.h>

namespace planner {
namespace base {
PlannerBase::PlannerBase(const uint32_t &dim, std::shared_ptr<NodeListBase> node_list)
    : terminate_search_cost_(0),
      constraint_(std::make_shared<ConstraintBase>(EuclideanSpace(dim))),
      node_list_(node_list) {}

PlannerBase::~PlannerBase() {}

void PlannerBase::setProblemDefinition(const std::shared_ptr<ConstraintBase> &constraint) {
  if (constraint->getDim() != constraint_->getDim()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "Problem definitions are invalid");
  }
  constraint_ = constraint;
  sampler_ = std::make_unique<Sampler>(constraint->space);
}

void PlannerBase::setTerminateSearchCost(const double &terminate_search_cost) {
  terminate_search_cost_ = terminate_search_cost;
}

const std::vector<State> &PlannerBase::getResult() const { return result_; }

double PlannerBase::getResultCost() const { return result_cost_; }

std::shared_ptr<NodeListBase> PlannerBase::getNodeList() const { return node_list_; }

std::shared_ptr<Node> PlannerBase::generateSteerNode(const std::shared_ptr<Node> &src_node,
                                                     const std::shared_ptr<Node> &dst_node,
                                                     const double &expand_dist) const {
  auto steered_node = std::make_shared<Node>(src_node->state, src_node, src_node->cost);
  auto dist_src_to_dst = src_node->state.distanceFrom(dst_node->state);
  if (dist_src_to_dst < expand_dist) {
    steered_node->cost += dist_src_to_dst;
    steered_node->state = dst_node->state;
  } else {
    steered_node->cost += expand_dist;
    steered_node->state = src_node->state + ((dst_node->state - src_node->state) / dist_src_to_dst) * expand_dist;
  }
  return steered_node;
}

void PlannerBase::updateParent(const std::shared_ptr<Node> &target_node,
                               const std::vector<std::shared_ptr<Node>> &near_nodes) const {
  auto min_cost_parent_node = target_node->parent;
  auto min_cost = std::numeric_limits<double>::max();
  for (const auto &near_node : near_nodes) {
    auto dist = target_node->state.distanceFrom(near_node->state);
    auto cost = near_node->cost + dist;
    if (cost < min_cost) {
      if (constraint_->checkCollision(target_node->state, near_node->state)) {
        min_cost_parent_node = near_node;
        min_cost = cost;
      }
    }
  }
  if (min_cost != std::numeric_limits<double>::max()) {
    target_node->parent = min_cost_parent_node;
    target_node->cost = min_cost;
  }
}

std::vector<std::shared_ptr<Node>> PlannerBase::rewireNearNodes(std::shared_ptr<Node> &new_node,
                                                                std::vector<std::shared_ptr<Node>> &near_nodes) const {
  std::vector<std::shared_ptr<Node>> rewired_nodes;
  for (const auto &near_node : near_nodes) {
    auto new_cost = new_node->cost + near_node->state.distanceFrom(new_node->state);
    if (new_cost < near_node->cost) {
      if (constraint_->checkCollision(new_node->state, near_node->state)) {
        near_node->parent = new_node;
        near_node->cost = new_cost;
        rewired_nodes.push_back(near_node);
      }
    }
  }
  return rewired_nodes;
}
}  // namespace base
}  // namespace planner
