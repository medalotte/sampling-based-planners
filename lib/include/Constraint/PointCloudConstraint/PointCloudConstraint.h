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

#ifndef LIB_INCLUDE_CONSTRAINT_POINTCLOUDCONSTRAINT_POINTCLOUDCONSTRAINT_H_
#define LIB_INCLUDE_CONSTRAINT_POINTCLOUDCONSTRAINT_POINTCLOUDCONSTRAINT_H_

#include <Constraint/ConstraintBase.h>
#include <State/State.h>

#include <algorithm>
#include <numeric>

namespace planner {

/**
 *  Super class of planner::ConstraintBase
 *  This class express constraint as set of hypersphere
 */
class PointCloudConstraint : public base::ConstraintBase {
 public:
  /**
   *  Hypersphere as a way of expressing of Obstacle
   */
  class Hypersphere {
   public:
    /**
     *  Constructor(Hypersphere)
     *  state and radius are initialize to zero
     */
    explicit Hypersphere(const uint32_t &dim);

    /**
     *  Constructor(Hypersphere)
     *  @state:  center of hypersphere
     *  @radius: radius of hypersphere
     *           (normalize to a positive decimal)
     */
    Hypersphere(const State &state, const double &radius);

    /**
     *  setter and getter function
     */
    void set(const State &state, const double &radius);
    void setState(const State &state);
    void setRadius(const double &radius);
    State getState() const;
    double getRadius() const;

   private:
    State state_;
    double radius_;
  };

  /**
   *  Constructor(PointCloudConstraint)
   *  @space: target space
   */
  explicit PointCloudConstraint(const EuclideanSpace &space);

  /**
   *  Constructor(PointCloudConstraint)
   *  @space:      target space
   *  @constraint: constraint that express set of hypersphere
   *               if dimension of hypersphere different from dimension of
   * space, this constructor throw std::invalid_argument
   */
  PointCloudConstraint(const EuclideanSpace &space, const std::vector<Hypersphere> &constraint);

  ~PointCloudConstraint() override;

  void set(const std::vector<Hypersphere> &constraint);

  const std::vector<Hypersphere> &getRef() const;

  bool checkCollision(const State &src, const State &dst) const override;

  ConstraintType checkConstraintType(const State &state) const override;

 private:
  std::vector<Hypersphere> constraint_;
};
}  // namespace planner

#endif /* LIB_INCLUDE_CONSTRAINT_POINTCLOUDCONSTRAINT_POINTCLOUDCONSTRAINT_H_ \
        */
