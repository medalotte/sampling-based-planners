/**
 *  MIT License
 *
 *  Copyright (c) 2019 Yuya Kudo
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#include <Planner/RRT/RRT.h>

namespace planner {
    RRT::RRT(uint32_t dim,
             uint32_t max_sampling_num,
             double goal_sampling_rate,
             double expand_dist) :
        base::PlannerBase(dim),
        max_sampling_num_(max_sampling_num),
        expand_dist_(expand_dist) {
        setGoalSamplingRate(goal_sampling_rate);
    }

    RRT::~RRT() {
    }

    void RRT::setMaxSamplingNum(uint32_t max_sampling_num) noexcept {
        max_sampling_num_ = max_sampling_num;
    }

    void RRT::setGoalSamplingRate(double goal_sampling_rate) {
        if(!(0.0 <= goal_sampling_rate && goal_sampling_rate <= 1.0)) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                     "Rate of Sampling goal state is invalid");
        }

        goal_sampling_rate_ = goal_sampling_rate;
    }

    void RRT::setExpandDist(double expand_dist) noexcept {
        expand_dist_ = expand_dist;
    }

    bool RRT::solve(const State& start, const State& goal) {
        // definition of random device
        std::random_device rand_dev;
        std::default_random_engine rand(rand_dev());

        // definition of constraint of generating random value in euclidean space
        std::vector<std::uniform_real_distribution<double>> rand_restrictions;
        for(size_t di = 1; di <= constraint_->space.getDim(); di++) {
            rand_restrictions.emplace_back(constraint_->space.getBound(di).low,
                                           constraint_->space.getBound(di).high);
        }

        // definition of random device in order to sample goal state with a certain probability
        auto sample_restriction = std::uniform_real_distribution<>(0, 1.0);

        // initialize list of node
        std::vector<std::shared_ptr<Node>> node_list;
        node_list.reserve(max_sampling_num_);
        node_list.push_back(std::make_shared<Node>(start, nullptr));

        // sampling on euclidean space
        uint32_t sampling_cnt = 0;
        while(true) {
            auto rand_node = std::make_shared<Node>(goal, nullptr);
            if(goal_sampling_rate_ < sample_restriction(rand)) {
                for(size_t i = 0; i < constraint_->space.getDim(); i++) {
                    rand_node->state.vals[i] = rand_restrictions[i](rand);
                }

                // resample when node do not meet constraint
                if(constraint_->checkConstraintType(rand_node->state) == ConstraintType::NOENTRY) {
                    continue;
                }
            }

            // get index of node that nearest node from sampling node
            size_t nearest_node_index = getNearestNodeIndex(rand_node, node_list);

            // generate new node
            auto new_node = generateSteerNode(node_list[nearest_node_index], rand_node, expand_dist_);

            // add to list if new node meets constraint
            if(checkCollision(node_list[nearest_node_index], new_node)) {
                node_list.push_back(new_node);

                // terminate processing if distance between new node and goal state is less than 'expand_dist'
                if(new_node->state.distanceFrom(goal) <= expand_dist_) {
                    node_list.push_back(std::make_shared<Node>(goal, node_list.back()));
                    break;
                }
            }

            sampling_cnt++;
            if(max_sampling_num_ == sampling_cnt) {
                return false;
            }
        }

        // store the result
        result_.clear();
        std::shared_ptr<base::NodeBase> result_node = node_list.back();
        while(true) {
            auto result_begin_itr = result_.begin();
            result_.insert(result_begin_itr, result_node->state);

            if(result_node->parent == nullptr) {
                break;
            }
            result_node = result_node->parent;
        }

        // store the node list
        node_list_.clear();
        std::move(node_list.begin(), node_list.end(), std::back_inserter(node_list_));

        return true;
    }

    size_t RRT::getNearestNodeIndex(const std::shared_ptr<Node>& target_node,
                                    const std::vector<std::shared_ptr<Node>>& node_list) const {
        size_t min_dist_index = 0;
        double min_dist = std::numeric_limits<double>::max();
        for(size_t i = 0; i < node_list.size(); i++) {
            double dist = node_list[i]->state.distanceFrom(target_node->state);
            if(dist < min_dist) {
                min_dist = dist;
                min_dist_index = i;
            }
        }

        return min_dist_index;
    }

    std::shared_ptr<RRT::Node> RRT::generateSteerNode(const std::shared_ptr<Node>& src_node,
                                                      const std::shared_ptr<Node>& dst_node,
                                                      const double& expand_dist) const {
        auto steered_node = std::shared_ptr<Node>(new Node{src_node->state, src_node});

        if(src_node->state.distanceFrom(dst_node->state) < expand_dist) {
            steered_node->state = dst_node->state;
        }
        else {
            auto src = src_node->state;
            auto dst = dst_node->state;

            double dim_expand_dist = expand_dist;
            for(int i = constraint_->space.getDim() - 1; 0 < i; i--) {
                double dist_delta_dim = dst.vals.back() - src.vals.back();
                src.vals.pop_back();
                dst.vals.pop_back();
                double dist_lower_dim = (i != 1) ? dst.distanceFrom(src) : dst.vals.front() - src.vals.front();

                double t = std::atan2(dist_delta_dim, dist_lower_dim);

                steered_node->state.vals[i] += dim_expand_dist * std::sin(t);
                dim_expand_dist              = dim_expand_dist * std::cos(t);
            }
            steered_node->state.vals[0] += dim_expand_dist;
        }

        return steered_node;
    }

    bool RRT::checkCollision(const std::shared_ptr<Node>& src_node,
                             const std::shared_ptr<Node>& dst_node) const {

        const auto vec = dst_node->state - src_node->state;
        for(double ratio_i = 0; ratio_i < 1.0; ratio_i += 0.1) {
            auto target = src_node->state + (vec * ratio_i);
            if(constraint_->checkConstraintType(target) == ConstraintType::NOENTRY) {
                return false;
            }
        }

        return true;
    }
}
