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

#ifndef LIB_INCLUDE_PLANNER_PLANNER_H_
#define LIB_INCLUDE_PLANNER_PLANNER_H_

#include <memory>

#include <State/State.h>
#include <Constraint/ConstraintBase.h>

namespace planner {
    namespace base {
        class PlannerBase {
        public:
            PlannerBase(uint32_t dim);
            ~PlannerBase();

            PlannerBase(const PlannerBase& obj) = delete;
            PlannerBase &operator =(const PlannerBase& obj) = delete;

            /**
             *  パスプランニングを実行する
             *  @start:         開始状態
             *  @goal:          終了状態
             */
            virtual bool solve(const State& start,
                               const State& goal) = 0;

            void setProblemDefinition(const std::shared_ptr<ConstraintBase>& constraint);

            std::vector<State> getResult() const;

        protected:
            std::vector<State>              result_;
            std::shared_ptr<ConstraintBase> constraint_;
        };
    }
}

#endif /* LIB_INCLUDE_PLANNER_PLANNER_H_ */
