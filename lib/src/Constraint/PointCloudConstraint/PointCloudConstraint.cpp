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

#include <Constraint/PointCloudConstraint/PointCloudConstraint.h>

namespace planner {
PointCloudConstraint::Hypersphere::Hypersphere(const uint32_t &dim) : state_(dim), radius_(0) {}

PointCloudConstraint::Hypersphere::Hypersphere(const State &state, const double &radius)
    : state_(state), radius_(std::abs(radius)) {}

void PointCloudConstraint::Hypersphere::set(const State &state, const double &radius) {
  state_ = state;
  radius_ = std::abs(radius);
}

void PointCloudConstraint::Hypersphere::setRadius(const double &radius) { radius_ = std::abs(radius); }

void PointCloudConstraint::Hypersphere::setState(const State &state) { state_ = state; }

State PointCloudConstraint::Hypersphere::getState() const { return state_; }

double PointCloudConstraint::Hypersphere::getRadius() const { return radius_; }

PointCloudConstraint::PointCloudConstraint(const EuclideanSpace &space) : base::ConstraintBase(space) {}

PointCloudConstraint::PointCloudConstraint(const EuclideanSpace &space, const std::vector<Hypersphere> &constraint)
    : base::ConstraintBase(space) {
  set(constraint);
}

PointCloudConstraint::~PointCloudConstraint() {}

void PointCloudConstraint::set(const std::vector<Hypersphere> &constraint) {
  for (const auto &data : constraint) {
    if (getDim() != data.getState().getDim()) {
      throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "State dimension is invalid");
    }
  }

  constraint_ = constraint;
}

const std::vector<PointCloudConstraint::Hypersphere> &PointCloudConstraint::getRef() const { return constraint_; }

bool PointCloudConstraint::checkCollision(const State &src, const State &dst) const {
  if (src.getDim() != dst.getDim() || getDim() != src.getDim()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "State dimension is invalid");
  }

  // return NOENTRY Type if the state is out of range
  for (size_t i = 0; i < getDim(); i++) {
    auto bound = space.getBound(i + 1);

    // return NOENTRY Type if the state is out of range
    if (src.vals[i] < bound.low || bound.high < src.vals[i] || dst.vals[i] < bound.low || bound.high < dst.vals[i]) {
      return false;
    }
  }

  auto dist = src.distanceFrom(dst);
  for (const auto &data : constraint_) {
    std::vector<double> sides{dist, src.distanceFrom(data.getState()), dst.distanceFrom(data.getState())};
    std::sort(sides.begin(), sides.end());

    // calc most minimum distance between a state on the line and the center of
    // the hypersphere
    auto min_dist_from_line = std::numeric_limits<double>::max();

    // when triangle is sharp or most long side is "src-dst"
    if (sides[2] == dist || std::pow(sides[2], 2) <= std::pow(sides[1], 2) + std::pow(sides[0], 2)) {
      // calc area of ​​the triangle by using Heron's formula
      auto s = std::accumulate(sides.begin(), sides.end(), 0.0) / 2.0;
      auto S = std::sqrt(s * (s - sides[0]) * (s - sides[1]) * (s - sides[2]));

      min_dist_from_line = (S * 2) / dist;
    } else {
      for (const auto &side : sides) {
        if (side != dist) {
          min_dist_from_line = std::min(min_dist_from_line, side);
        }
      }
    }

    if (min_dist_from_line <= data.getRadius()) {
      return false;
    }
  }
  return true;
}

ConstraintType PointCloudConstraint::checkConstraintType(const State &state) const {
  if (getDim() != state.getDim()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "State dimension is invalid");
  }

  // return NOENTRY Type if the state is out of range
  for (size_t i = 0; i < getDim(); i++) {
    auto bound = space.getBound(i + 1);

    // return NOENTRY Type if the state is out of range
    if (state.vals[i] < bound.low || bound.high < state.vals[i]) {
      return ConstraintType::NOENTRY;
    }
  }

  for (const auto &data : constraint_) {
    if (state.distanceFrom(data.getState()) < data.getRadius()) {
      return ConstraintType::NOENTRY;
    }
  }

  return ConstraintType::ENTAERABLE;
}
}  // namespace planner
