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

#include <Planner/InformedRRTStar/InformedRRTStar.h>

namespace planner {
InformedRRTStar::InformedRRTStar(const uint32_t &dim, const uint32_t &max_sampling_num,
                                 const double &goal_sampling_rate, const double &expand_dist, const double &R,
                                 const double &goal_region_radius)
    : base::PlannerBase(dim, std::make_shared<KDTreeNodeList>(dim)),
      max_sampling_num_(max_sampling_num),
      expand_dist_(expand_dist),
      R_(R),
      goal_region_radius_(goal_region_radius) {
  setGoalSamplingRate(goal_sampling_rate);
}

InformedRRTStar::~InformedRRTStar() {}

void InformedRRTStar::setMaxSamplingNum(const uint32_t &max_sampling_num) { max_sampling_num_ = max_sampling_num; }

void InformedRRTStar::setGoalSamplingRate(const double &goal_sampling_rate) {
  if (!(0.0 <= goal_sampling_rate && goal_sampling_rate <= 1.0)) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                "Rate of Sampling goal state is invalid");
  }

  goal_sampling_rate_ = goal_sampling_rate;
}

void InformedRRTStar::setExpandDist(const double &expand_dist) { expand_dist_ = expand_dist; }

void InformedRRTStar::setR(const double &R) { R_ = R; }

void InformedRRTStar::setGoalRegionRadius(const double &goal_region_radius) {
  goal_region_radius_ = goal_region_radius;
}

bool InformedRRTStar::solve(const State &start, const State &goal) {
  auto estimate_cost = [](const std::shared_ptr<Node> &node) -> double { return node->cost + node->cost_to_goal; };

  // initialize sampler and node list
  sampler_->applyStartAndGoal(start, goal);
  node_list_->init();
  node_list_->add(std::make_shared<Node>(start, nullptr));

  // sampling on euclidean space
  std::shared_ptr<Node> min_cost_node = nullptr;
  for (size_t i = 0; i < max_sampling_num_; i++) {
    // sampling node
    auto rand_node = std::make_shared<Node>(goal, nullptr, 0);
    if (goal_sampling_rate_ < sampler_->getUniformUnitRandomVal()) {
      if (min_cost_node == nullptr) {
        rand_node->state = sampler_->run(Sampler::Mode::WholeArea);
      } else {
        sampler_->setBestCost(min_cost_node->cost + goal.distanceFrom(min_cost_node->state));
        rand_node->state = sampler_->run(Sampler::Mode::HeuristicDomain);
      }

      // resample when rand node dose not meet constraint
      if (constraint_->checkConstraintType(rand_node->state) == ConstraintType::NOENTRY) {
        continue;
      }
    }

    // get node that is nearest neighbor node from node list and generate new
    // node
    auto nearest_node = node_list_->searchNN(rand_node);
    auto new_node = generateSteerNode(nearest_node, rand_node, expand_dist_);
    new_node->cost_to_goal = new_node->state.distanceFrom(goal);

    // add to list if new node meets constraint
    if (constraint_->checkCollision(nearest_node->state, new_node->state)) {
      // find nodes that exist on certain domain
      auto nof_node = node_list_->getSize();
      auto radius =
          std::min(expand_dist_, R_ * std::pow((std::log(nof_node) / nof_node), 1.0 / constraint_->space.getDim()));
      auto near_nodes = node_list_->searchNBHD(new_node, radius);

      // choose parent node of new node from near nodes
      updateParent(new_node, near_nodes);

      // add new node to list
      node_list_->add(new_node);

      // redefine parent node of near nodes
      auto changed_cost_nodes = rewireNearNodes(new_node, near_nodes);

      // reacquire the lowest cost node close to the goal
      changed_cost_nodes.push_back(new_node);
      for (const auto &changed_cost_node : changed_cost_nodes) {
        if (changed_cost_node->cost_to_goal <= goal_region_radius_) {
          if (min_cost_node == nullptr || estimate_cost(changed_cost_node) < estimate_cost(min_cost_node)) {
            if (constraint_->checkCollision(goal, changed_cost_node->state)) {
              min_cost_node = changed_cost_node;
            }
          }
        }
      }

      if (min_cost_node != nullptr && estimate_cost(min_cost_node) < terminate_search_cost_) {
        break;
      }
    }
  }

  // store the result
  result_.clear();
  if (min_cost_node == nullptr) {
    return false;
  } else {
    auto result_node = min_cost_node;
    result_cost_ = estimate_cost(result_node);
    if (result_node->state != goal) {
      result_.push_back(goal);
    }

    while (true) {
      result_.insert(result_.begin(), result_node->state);
      if (result_node->parent == nullptr) {
        break;
      }

      result_node = result_node->parent;
    }
    return true;
  }
}
}  // namespace planner
