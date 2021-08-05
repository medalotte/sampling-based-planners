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

#include <Constraint/ConstraintBase.h>

namespace planner {
namespace base {
ConstraintBase::ConstraintBase(const EuclideanSpace &_space) : space(_space) {}

ConstraintBase::~ConstraintBase() {}

uint32_t ConstraintBase::getDim() const { return space.getDim(); }

bool ConstraintBase::checkCollision(const State &src, const State &dst) const {
  return (checkConstraintType(src) == ConstraintType::ENTAERABLE &&
          checkConstraintType(dst) == ConstraintType::ENTAERABLE)
             ? true
             : false;
}

ConstraintType ConstraintBase::checkConstraintType(const State &state) const {
  for (size_t i = 0; i < state.getDim(); i++) {
    auto bound = space.getBound(i + 1);

    // return NOENTRY Type if the state is out of range
    if (state.vals[i] < bound.low || bound.high < state.vals[i]) {
      return ConstraintType::NOENTRY;
    }
  }

  return ConstraintType::ENTAERABLE;
}
}  // namespace base
}  // namespace planner
