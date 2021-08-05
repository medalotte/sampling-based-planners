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

#ifndef LIB_INCLUDE_SAMPLER_SAMPLER_H_
#define LIB_INCLUDE_SAMPLER_SAMPLER_H_

#include <EuclideanSpace/EuclideanSpace.h>
#include <State/State.h>

#include <Eigen/Dense>
#include <cstdint>
#include <random>

namespace planner {
class Sampler {
 public:
  enum class Mode { WholeArea, HeuristicDomain };

  explicit Sampler(const EuclideanSpace &space);
  Sampler(const EuclideanSpace &space, const State &start, const State &goal,
          const double &best_cost = std::numeric_limits<double>::max());
  ~Sampler();

  void applyStartAndGoal(const State &start, const State &goal);

  void setBestCost(const double &best_cost);

  /**
   *  get unit uniform distribution value (i.e., return between 0.0 and 1.0)
   *  @Return: random value
   */
  double getUniformUnitRandomVal();

  /**
   *  sample a ramdom state from certain area
   *  @mode: Mode::WholeArea:       sample a random state from whole area
   *         Mode::HeuristicDomain: sample a random state from heuristic domain
   *  @Return: sampled state
   */
  State run(const Mode &mode = Mode::WholeArea);

 private:
  const uint32_t dim_;

  std::mt19937 rand_;
  std::normal_distribution<> dist_gauss_;
  std::uniform_real_distribution<> dist_unit_;
  std::vector<std::uniform_real_distribution<>> dist_space_;

  double min_cost_;
  double best_cost_;
  Eigen::MatrixXd rotate_mat_;
  Eigen::VectorXd center_state_;

  /**
   *  generate distribution for sampling a random state from whole area
   *  @space:  target space
   *  @Return: uniform distribution object at each dimension
   */
  std::vector<std::uniform_real_distribution<>> generateSpaceDistribution(const EuclideanSpace &space) const;

  /**
   *  calculate rotation matrix using singular value decomposition
   *  ${\bf C}={\bf U}diag(1,\ldots,1,\det({\bf U})\det({\bf V})){\bf V}^{T}$
   *  ${\bf U}\sum{}{\bf V}^{T}\equiv{\bf M}$
   *  ${\bf M}={\bf a}_{1}{\bf 1}_{1}^{T}$
   *  ${\bf a}_{1}=({\bf x}_{goal}-{\bf x}_{start})/\| {\bf x}_{goal}-{\bf
   * x}_{start} \|_2$
   *
   *  @start:  start state
   *  @goal:   goal state
   *  @Return: rotation matrix
   */
  Eigen::MatrixXd calcRotationToWorldFlame(const State &start, const State &goal) const;

  /**
   *  random sampling on unit n-ball
   *  @dim:    dimension
   *  @Return: random state on unit n-ball
   */
  State sampleUnitNBall(const uint32_t &dim);
};
}  // namespace planner

#endif /* LIB_INCLUDE_SAMPLER_SAMPLER_H_ */
