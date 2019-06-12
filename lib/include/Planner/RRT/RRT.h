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

#ifndef LIB_INCLUDE_PLANNER_RRT_RRT_H_
#define LIB_INCLUDE_PLANNER_RRT_RRT_H_

#include <iostream>
#include <vector>
#include <random>
#include <cstdint>
#include <cmath>
#include <functional>
#include <Planner/PlannerBase.h>

namespace planner {
    class RRT : public base::PlannerBase {
        struct Node {
            State                 state;
            std::shared_ptr<Node> parent;
        };

    public:
        RRT(uint32_t dim,
            uint32_t max_sampling_num   = 10000,
            double   goal_sampling_rate = 0.05,
            double   expand_dist        = 1.0);
        ~RRT();

        void setMaxSamplingNum(uint32_t max_sampling_num) noexcept;
        void setGoalSamplingRate(double goal_sampling_rate);
        void setExpandDist(double expand_dist) noexcept;

        bool solve(const State& start, const State& goal) override;

    private:
        uint32_t max_sampling_num_;
        double   goal_sampling_rate_;
        double   expand_dist_;

        /**
         *  Get index that nearest node from target node in 'node_list'
         *  @target_node: target node
         *  @node_list:   list that contein existing node
         *  @Return:      index of nearest node in 'node_list'
         */
        size_t getNearestNodeIndex(const std::shared_ptr<Node>&              target_node,
                                   const std::vector<std::shared_ptr<Node>>& node_list) const;

        /**
         *  Generate Steered node that is 'expand_dist' away from 'src_node' to 'dst_node' direction
         *  @src_node:    source node
         *  @dst_node:    destination node
         *  @expand_dist: distance from 'src_node' of steered node
         *  @Return:      steered node
         */
        std::shared_ptr<Node> generateSteerNode(const std::shared_ptr<Node>& src_node,
                                                const std::shared_ptr<Node>& dst_node,
                                                const double&                expand_dist) const;

        /**
         *  Check a constraint between 'src_node' and 'dst_node'
         *  @src_node: source node
         *  @dst_node: destination node
         *  @Return:   If the path of between 'src_node' and 'dst_node' entry ConstraintType::NOENTRY,
         *             return false
         */
        bool checkCollision(const std::shared_ptr<Node>& src_node,
                            const std::shared_ptr<Node>& dst_node) const;
    };
}

#endif /* LIB_INCLUDE_PLANNER_RRT_RRT_H_ */
