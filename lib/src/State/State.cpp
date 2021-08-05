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

#include <State/State.h>

namespace planner {

State::State(uint32_t dim) {
  if (dim == 0) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "Can not set zero-dimension state");
  }

  vals = std::vector<double>(dim);
}

State::State(const std::vector<double> &_vals) {
  if (_vals.size() == 0) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "Can not set zero-dimension state");
  }

  vals = _vals;
}

State::~State() {}

uint32_t State::getDim() const { return vals.size(); }

double State::norm() const { return std::sqrt(dot(*this)); }

double State::dot(const State &other) const {
  if (vals.size() != other.vals.size()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                "Can not calculate because dimension is different from");
  }

  double ret = 0;
  for (size_t i = 0; i < vals.size(); i++) {
    ret += vals[i] * other.vals[i];
  }

  return ret;
}

double State::distanceFrom(const State &other) const { return (other - *this).norm(); }

State State::normalized() const { return *this / norm(); }

bool State::isZero() const {
  for (const auto &val : vals) {
    if (val != 0) {
      return false;
    }
  }

  return true;
}

State State::operator+() const { return *this; }

State State::operator-() const {
  State ret = *this;
  for (auto &val : ret.vals) {
    val *= -1;
  }

  return ret;
}

State State::operator+(const State &other) const {
  if (vals.size() != other.vals.size()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                "Can not calculate because dimension is different from");
  }

  State ret = *this;
  for (size_t i = 0; i < ret.getDim(); i++) {
    ret.vals[i] += other.vals[i];
  }

  return ret;
}

State State::operator-(const State &other) const {
  if (vals.size() != other.vals.size()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                "Can not calculate because dimension is different from");
  }

  State ret = *this;
  for (size_t i = 0; i < ret.getDim(); i++) {
    ret.vals[i] -= other.vals[i];
  }

  return ret;
}

bool State::operator==(const State &other) const {
  if (vals.size() != other.vals.size()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                "Can not calculate because dimension is different from");
  }

  for (size_t i = 0; i < getDim(); i++) {
    if (vals[i] != other.vals[i]) {
      return false;
    }
  }

  return true;
}

bool State::operator!=(const State &other) const {
  if (vals.size() != other.vals.size()) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                "Can not calculate because dimension is different from");
  }

  for (size_t i = 0; i < getDim(); i++) {
    if (vals[i] != other.vals[i]) {
      return true;
    }
  }

  return false;
}

State State::operator*(double s) const {
  State ret = *this;
  for (size_t i = 0; i < ret.getDim(); i++) {
    ret.vals[i] *= s;
  }

  return ret;
}

State State::operator/(double s) const {
  State ret = *this;
  for (size_t i = 0; i < ret.getDim(); i++) {
    ret.vals[i] /= s;
  }

  return ret;
}

std::ostream &operator<<(std::ostream &os, const State &obj) {
  for (size_t i = 0; i < obj.getDim(); i++) {
    os << "[" << i << "] " << obj.vals[i];
    if (i != obj.getDim()) {
      os << ", ";
    }
  }
  return os;
};
}  // namespace planner
