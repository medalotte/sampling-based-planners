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

#include <Constraint/GridConstraint/GridConstraint.h>

namespace planner {
GridConstraint::GridConstraint(const EuclideanSpace &space) : base::ConstraintBase(space) {}

GridConstraint::GridConstraint(const EuclideanSpace &space, const std::vector<ConstraintType> &constraint,
                               const std::vector<uint32_t> &each_dim_size)
    : base::ConstraintBase(space) {
  set(constraint, each_dim_size);
}

GridConstraint::~GridConstraint() {}

void GridConstraint::set(const std::vector<ConstraintType> &constraint, const std::vector<uint32_t> &each_dim_size) {
  constraint_ = constraint;
  each_dim_size_ = each_dim_size;
}

const std::vector<ConstraintType> &GridConstraint::getConstraintRef() const { return constraint_; }

const std::vector<uint32_t> &GridConstraint::getEachDimSizeRef() const { return each_dim_size_; }

State GridConstraint::calcGridIdx(const State &state) const {
  if (getDim() != state.getDim()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "State dimension is invalid");
  }

  State idx(state.getDim());
  for (size_t i = 0; i < state.getDim(); i++) {
    auto bound = space.getBound(i + 1);

    // return -1 if the state is out of range
    if (state.vals[i] < bound.low || bound.high < state.vals[i]) {
      return idx;
    } else {
      idx.vals[i] = (state.vals[i] - bound.low) * each_dim_size_[i] / bound.getRange();
    }
  }

  return idx;
}

std::vector<std::vector<uint32_t>> GridConstraint::calcLineIndices(State src_idx, State dst_idx) const {
  if (src_idx.getDim() != dst_idx.getDim() || getDim() != src_idx.getDim()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "Idx dimension is invalid");
  }

  const auto dst_idx_orig = dst_idx;

  // calculate all grid indices between the src and the dst by n-dimension
  // Bresenham's line algorithm
  std::vector<std::vector<uint32_t>> line_indices;
  const auto plot = [&](const double x, const std::vector<double> &idx) -> void {
    std::vector<uint32_t> floored_idx(idx.size() + 1);
    floored_idx[0] = std::floor(x);
    for (size_t i = 0; i < idx.size(); i++) {
      floored_idx[i + 1] = std::floor(idx[i]);
    }
    line_indices.push_back(floored_idx);
  };

  std::vector<bool> swap_dim(getDim() - 1, false);
  for (size_t i = 1; i < getDim(); i++) {
    swap_dim[i - 1] = std::fabs(dst_idx.vals[i] - src_idx.vals[i]) > std::fabs(dst_idx.vals[0] - src_idx.vals[0]);
    if (swap_dim[i - 1]) {
      std::swap(src_idx.vals[0], src_idx.vals[i]);
      std::swap(dst_idx.vals[0], dst_idx.vals[i]);
    }
  }

  const auto idx_diff = dst_idx - src_idx;

  std::vector<double> delta(getDim());
  std::vector<double> step(getDim());
  for (size_t i = 0; i < getDim(); i++) {
    delta[i] = fabs(idx_diff.vals[i]);
    step[i] = (src_idx.vals[i] > dst_idx.vals[i]) ? -1 : 1;
  }

  std::vector<double> v(getDim() - 1);
  std::vector<double> drift(getDim() - 1);
  for (size_t i = 0; i < getDim() - 1; i++) {
    v[i] = src_idx.vals[i + 1];
    drift[i] = delta[0] / 2.0;
  }

  for (double x = src_idx.vals[0]; std::fabs(x - dst_idx.vals[0]) > 1.0; x += step[0]) {
    auto cx = x;
    auto cv = v;
    for (int i = getDim() - 2; i >= 0; i--) {
      if (swap_dim[i]) {
        std::swap(cv[i], cx);
      }
    }
    plot(cx, cv);
    for (size_t i = 0; i < getDim() - 1; i++) {
      drift[i] -= delta[i + 1];
      if (drift[i] < 0) {
        v[i] += step[i + 1];
        drift[i] += delta[0];
      }
    }
  }

  return line_indices;
}

bool GridConstraint::checkCollision(const State &src, const State &dst) const {
  if (src.getDim() != dst.getDim() || getDim() != src.getDim()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "State dimension is invalid");
  }
  const auto src_idx = calcGridIdx(src);
  const auto dst_idx = calcGridIdx(dst);
  for (size_t i = 0; i < getDim(); i++) {
    if (src_idx.vals[i] == -1 || dst_idx.vals[i] == -1) {
      return false;
    }
  }
  const auto line_indices = calcLineIndices(src_idx, dst_idx);
  for (const auto idx : line_indices) {
    if (checkConstraintType(idx) == ConstraintType::NOENTRY) {
      return false;
    }
  }
  return true;
}

ConstraintType GridConstraint::checkConstraintType(const State &state) const {
  if (getDim() != state.getDim()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "State dimension is invalid");
  }

  // get index on single dimension array which correspond with state
  uint32_t index = 0;
  for (size_t i = 0; i < state.getDim(); i++) {
    auto bound = space.getBound(i + 1);

    // return NOENTRY Type if the state is out of range
    if (state.vals[i] < bound.low || bound.high < state.vals[i]) {
      return ConstraintType::NOENTRY;
    }

    uint32_t product = 1;
    for (size_t j = 0; j < i; j++) {
      product *= each_dim_size_[j];
    }
    index += std::floor((state.vals[i] - bound.low) * each_dim_size_[i] / bound.getRange()) * product;
  }

  return constraint_[index];
}

ConstraintType GridConstraint::checkConstraintType(const std::vector<uint32_t> &idx) const {
  if (getDim() != idx.size()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "Idx dimension is invalid");
  }

  // get index on single dimension array which correspond with state
  uint32_t constraint_array_idx = 0;
  for (size_t i = 0; i < idx.size(); i++) {
    if (idx[i] < 0 || each_dim_size_[i] < idx[i]) {
      return ConstraintType::NOENTRY;
    }
    uint32_t product = 1;
    for (size_t j = 0; j < i; j++) {
      product *= each_dim_size_[j];
    }
    constraint_array_idx += idx[i] * product;
  }

  return constraint_[constraint_array_idx];
}
}  // namespace planner
