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

#ifndef LIB_INCLUDE_PLANNER_INFORMEDRRTSTAR_INFORMEDRRTSTAR_H_
#define LIB_INCLUDE_PLANNER_INFORMEDRRTSTAR_INFORMEDRRTSTAR_H_

#include <iostream>
#include <vector>
#include <random>
#include <cstdint>
#include <cmath>
#include <functional>
#include <Planner/PlannerBase.h>

#include <Eigen/Dense>

namespace planner {
    class InformedRRTStar : public base::PlannerBase {
        class Node : public base::NodeBase {
        public:
            double cost;
            Node(const State&                _state,
                 const std::shared_ptr<Node> _parent,
                 const double&               _cost) :
                base::NodeBase(_state, _parent), cost(_cost) {}
        };

    public:
        InformedRRTStar(const uint32_t& dim,
                        const uint32_t& max_sampling_num   = 1000,
                        const double&   goal_sampling_rate = 0.25,
                        const double&   expand_dist        = 1.0,
                        const double&   R                  = 10.0,
                        const double&   goal_region_radius = 10.0);
        ~InformedRRTStar();

        void setMaxSamplingNum(const uint32_t& max_sampling_num);
        void setGoalSamplingRate(const double& goal_sampling_rate);
        void setExpandDist(const double& expand_dist);
        void setR(const double& R);
        void setGoalRegionRadius(const double& goal_region_radius);

        bool solve(const State& start, const State& goal) override;

    private:
        uint32_t max_sampling_num_;
        double   goal_sampling_rate_;
        double   expand_dist_;
        double   R_;
        double   goal_region_radius_;

        /**
         *  Get index that nearest node from target node in 'node_list'
         *  @target_node: target node
         *  @node_list:   list that contein existing node
         *  @Return:      index of nearest node in 'node_list'
         */
        size_t getNearestNodeIndex(const std::shared_ptr<Node>& target_node,
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
         *  Find near nodes in 'node_list'
         *  Near nodes are definition as formula using target dimension and number of total node
         *  @target_node: target node
         *  @node_list:   list that contein existing node
         *  @Return:      set of indexes that are near 'target_node' in 'node_list'
         */
        std::vector<size_t> findNearNodes(const std::shared_ptr<Node>&              target_node,
                                          const std::vector<std::shared_ptr<Node>>& node_list) const;

        /**
         *  Choose parent node from near node that find in findNearNodes()
         *  @target_node:       target node
         *  @node_list:         list that contein existing node
         *  @near_node_indexes: return value of findNearNodes()
         *  @Return: node that choosed new parent node
         */
        std::shared_ptr<Node> chooseParentNode(const std::shared_ptr<Node>&              target_node,
                                               const std::vector<std::shared_ptr<Node>>& node_list,
                                               const std::vector<size_t>&                near_node_indexes) const;

        /**
         *  redefine parent node of near node that find in findNearNodes()
         *  @node_list:         list that contein existing node
         *  @near_node_indexes: return value of findNearNodes()
         *  @Return:            void
         */
        void rewireNearNodes(std::vector<std::shared_ptr<Node>>& node_list,
                             const std::vector<size_t>&          near_node_indexes) const;

        /**
         *  get most low cost node in 'node_list' within 'radius' from 'target_state'
         *  @target_state: target state
         *  @radius:       radius
         *  @node_list:    list that contein existing node
         *  @Return:       index of most low cost node in 'node_list'
         *                 If do NOT exist node in 'node_list' within 'radius',
         *                 return -1
         */
        int getBestNodeIndex(const State&                              target_state,
                             const double&                             radius,
                             const std::vector<std::shared_ptr<Node>>& node_list) const;

        /**
         *  calculate rotation matrix using singular value decomposition
         *  ${\bf C}={\bf U}diag(1,\ldots,1,\det({\bf U})\det({\bf V})){\bf V}^{T}$
         *  ${\bf U}\sum{}{\bf V}^{T}\equiv{\bf M}$
         *  ${\bf M}={\bf a}_{1}{\bf 1}_{1}^{T}$
         *  ${\bf a}_{1}=({\bf x}_{goal}-{\bf x}_{start})/\| {\bf x}_{goal}-{\bf x}_{start} \|_2$
         *
         *  @start:  start state
         *  @goal:   goal state
         *  @Return: rotation matrix
         */
        Eigen::MatrixXd calcRotationToWorldFlame(const State& start,
                                                 const State& goal) const;

        /**
         *  random sampling on unit n-ball
         *  @dim:    dimension
         *  @Return: random state on unit n-ball
         */
        State sampleUnitNBall(const uint32_t& dim) const;
    };
}

#endif /* LIB_INCLUDE_PLANNER_INFORMEDRRTSTAR_INFORMEDRRTSTAR_H_ */
