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

#ifndef LIB_INCLUDE_PLANNER_RRT_RRT_H_
#define LIB_INCLUDE_PLANNER_RRT_RRT_H_

#include <Node/KDTreeNodeList/KDTreeNodeList.h>
#include <Planner/PlannerBase.h>

#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <random>
#include <vector>

namespace planner {
class RRT : public base::PlannerBase {
 public:
  RRT(const uint32_t &dim, const uint32_t &max_sampling_num = 10000, const double &goal_sampling_rate = 0.05,
      const double &expand_dist = 1.0);
  ~RRT();

  void setMaxSamplingNum(uint32_t max_sampling_num) noexcept;
  void setGoalSamplingRate(double goal_sampling_rate);
  void setExpandDist(double expand_dist) noexcept;

  bool solve(const State &start, const State &goal) override;

 private:
  uint32_t max_sampling_num_;
  double goal_sampling_rate_;
  double expand_dist_;
};
}  // namespace planner

#endif /* LIB_INCLUDE_PLANNER_RRT_RRT_H_ */
