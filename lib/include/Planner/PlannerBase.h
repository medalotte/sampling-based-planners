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

#ifndef LIB_INCLUDE_PLANNER_PLANNERBASE_H_
#define LIB_INCLUDE_PLANNER_PLANNERBASE_H_

#include <Node/NodeBase.h>
#include <Constraint/ConstraintBase.h>
#include <Sampler/Sampler.h>

namespace planner {
    namespace base {
        /**
         *  Base class of planners which are sampling-based method
         */
        class PlannerBase {
        public:
            explicit PlannerBase(uint32_t dim);
            virtual ~PlannerBase();

            PlannerBase(const PlannerBase&) = delete;
            PlannerBase& operator =(const PlannerBase&) = delete;
            PlannerBase(PlannerBase&&) noexcept = delete;
            PlannerBase& operator =(PlannerBase&&) noexcept = delete;

            /**
             *  Execute path planning
             *  @start:  start state
             *  @goal:   goal state
             *  @Return: whether the path planning was successful
             */
            virtual bool solve(const State& start,
                               const State& goal) = 0;

            void setProblemDefinition(const std::shared_ptr<ConstraintBase>& constraint);

            void setTerminateSearchCost(const double& terminate_search_cost);

            const std::vector<State>& getResultRef() const;

            double getResultCost() const;

            const std::vector<std::shared_ptr<NodeBase>>& getNodeListRef() const;

        protected:
            std::vector<State>                     result_;
            double                                 result_cost_;
            double                                 terminate_search_cost_;
            std::vector<std::shared_ptr<NodeBase>> node_list_;
            std::shared_ptr<ConstraintBase>        constraint_;
            std::unique_ptr<Sampler>               sampler_;
        };
    }
}

#endif /* LIB_INCLUDE_PLANNER_PLANNERBASE_H_ */
