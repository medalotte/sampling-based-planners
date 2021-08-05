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

#ifndef LIB_INCLUDE_CONSTRAINT_CONSTRAINT_H_
#define LIB_INCLUDE_CONSTRAINT_CONSTRAINT_H_

#include <EuclideanSpace/EuclideanSpace.h>
#include <State/State.h>

#include <cstdint>

namespace planner {

/**
 *  definition of type of constraint
 */
enum class ConstraintType { NOENTRY, ENTAERABLE };

namespace base {

/**
 *  Base class of constraint for planner
 */
class ConstraintBase {
 public:
  EuclideanSpace space;

  /**
   *  Constructor(ConstraintBase)
   *  @space: target space
   */
  explicit ConstraintBase(const EuclideanSpace &_space);
  virtual ~ConstraintBase();

  uint32_t getDim() const;

  /**
   *  Check whether collision occurred between src and dst
   *  @src:    source state
   *  @dst:    destination state
   *  @Return: If the path of between 'src' and 'dst' entry
   * ConstraintType::NOENTRY, return false
   */
  virtual bool checkCollision(const State &src, const State &dst) const;

  /**
   *  Check constraint at given state
   *  @state: target state
   *  @Return: type of constraint at target state
   */
  virtual ConstraintType checkConstraintType(const State &state) const;
};
}  // namespace base
}  // namespace planner

#endif /* LIB_INCLUDE_CONSTRAINT_CONSTRAINT_H_ */
