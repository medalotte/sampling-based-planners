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

#ifndef LIB_INCLUDE_EUCLIDEANSPACE_EUCLIDEANSPACE_H_
#define LIB_INCLUDE_EUCLIDEANSPACE_EUCLIDEANSPACE_H_

#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

namespace planner {

/**
 *  Express bound in euclidean space
 */
class Bound {
 public:
  double low;
  double high;

  Bound();
  Bound(const double &low, const double &high);

  ~Bound();

  double getRange() const;
};

/**
 *  Express euclidean space
 */
class EuclideanSpace {
 public:
  explicit EuclideanSpace(const uint32_t &dim);

  ~EuclideanSpace();

  uint32_t getDim() const noexcept;

  void setBound(std::vector<Bound> &bounds);

  Bound getBound(const uint32_t &dim) const;

  const std::vector<Bound> &getBoundsRef() const;

 private:
  uint32_t dim_;
  std::vector<Bound> bounds_;
};
}  // namespace planner

#endif /* LIB_INCLUDE_EUCLIDEANSPACE_EUCLIDEANSPACE_H_ */
