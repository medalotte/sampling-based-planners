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

#ifndef LIB_INCLUDE_CONSTRAINT_GRIDCONSTRAINT_GRIDCONSTRAINT_H_
#define LIB_INCLUDE_CONSTRAINT_GRIDCONSTRAINT_GRIDCONSTRAINT_H_

#include <Constraint/ConstraintBase.h>
#include <State/State.h>

#include <cstdint>
#include <iostream>
#include <vector>

namespace planner {

/**
 *  Super class of planner::ConstraintBase
 *  This class express constraint as like an image or multidimensional array
 *  and nomalize automatically for space size when call checkConstraintType()
 */
class GridConstraint : public base::ConstraintBase {
 public:
  /**
   *  Constructor(GridConstraint)
   *  @space: target space
   */
  explicit GridConstraint(const EuclideanSpace &space);

  /**
   *  Constructor(GridConstraint)
   *  @space:         target space
   *  @constraint:    multidimensional array express as one dimensional array
   *                  (e.g. '(x, y)' -> 'x + y * x_size' where 2 dimensions)
   *  @each_dim_size: each dimension size of constraint you set
   */
  GridConstraint(const EuclideanSpace &space, const std::vector<ConstraintType> &constraint,
                 const std::vector<uint32_t> &each_dim_size);

  ~GridConstraint() override;

  void set(const std::vector<ConstraintType> &constraint, const std::vector<uint32_t> &each_dim_size);

  const std::vector<ConstraintType> &getConstraintRef() const;

  const std::vector<uint32_t> &getEachDimSizeRef() const;

  State calcGridIdx(const State &state) const;

  std::vector<std::vector<uint32_t>> calcLineIndices(State src_idx, State dst_idx) const;

  bool checkCollision(const State &src, const State &dst) const override;

  ConstraintType checkConstraintType(const State &state) const override;

  ConstraintType checkConstraintType(const std::vector<uint32_t> &idx) const;

 private:
  std::vector<ConstraintType> constraint_;
  std::vector<uint32_t> each_dim_size_;
};
}  // namespace planner

#endif /* LIB_INCLUDE_CONSTRAINT_GRIDCONSTRAINT_GRIDCONSTRAINT_H_ */
