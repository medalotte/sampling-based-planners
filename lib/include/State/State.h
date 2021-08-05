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

#ifndef LIB_INCLUDE_STATE_STATE_H_
#define LIB_INCLUDE_STATE_STATE_H_

#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <vector>

namespace planner {

/**
 *  Express state on multidimensional
 */
class State {
 public:
  std::vector<double> vals;

  explicit State(uint32_t dim);

  explicit State(const std::vector<double> &_vals);

  template <class... A>
  State(const A &... _vals) : vals(std::initializer_list<double>{_vals...}) {}

  ~State();

  uint32_t getDim() const;

  double norm() const;

  double dot(const State &other) const;

  double distanceFrom(const State &other) const;

  State normalized() const;

  bool isZero() const;

  State operator+() const;
  State operator-() const;
  State operator+(const State &other) const;
  State operator-(const State &other) const;
  bool operator==(const State &other) const;
  bool operator!=(const State &other) const;
  State operator*(double s) const;
  State operator/(double s) const;

  friend std::ostream &operator<<(std::ostream &os, const State &obj);
};
}  // namespace planner

#endif /* LIB_INCLUDE_STATE_STATE_H_ */
