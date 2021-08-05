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

#include <EuclideanSpace/EuclideanSpace.h>

namespace planner {
Bound::Bound() : low(0), high(0) {}

Bound::Bound(const double &_low, const double &_high) {
  if (_high < _low) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "Bound is invalid");
  }

  high = _high;
  low = _low;
}

Bound::~Bound() {}

double Bound::getRange() const { return high - low; }

EuclideanSpace::EuclideanSpace(const uint32_t &dim) {
  if (dim == 0) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "Dimension is invalid");
  }
  dim_ = dim;
  bounds_ = std::vector<Bound>(dim, Bound(0, 0));
}

EuclideanSpace::~EuclideanSpace() {}

uint32_t EuclideanSpace::getDim() const noexcept { return dim_; }

void EuclideanSpace::setBound(std::vector<Bound> &bounds) {
  if (bounds.size() != dim_) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "Condition of bound is invalid");
  }
  bounds_ = bounds;
}

Bound EuclideanSpace::getBound(const uint32_t &dim) const {
  if (!(0 < dim && dim <= dim_)) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "Dimention is out of range");
  }
  return bounds_[dim - 1];
}

const std::vector<Bound> &EuclideanSpace::getBoundsRef() const { return bounds_; }
}  // namespace planner
