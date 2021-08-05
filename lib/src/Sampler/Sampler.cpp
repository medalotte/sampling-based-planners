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

#include <Sampler/Sampler.h>

namespace planner {
Sampler::Sampler(const EuclideanSpace &space)
    : dim_(space.getDim()),
      rand_(std::mt19937(std::random_device()())),
      dist_gauss_(std::normal_distribution<>(0.0, 1.0)),
      dist_unit_(std::uniform_real_distribution<>(0.0, 1.0)),
      dist_space_(generateSpaceDistribution(space)),
      min_cost_(0),
      best_cost_(0),
      rotate_mat_(Eigen::MatrixXd::Identity(space.getDim(), space.getDim())),
      center_state_(Eigen::VectorXd::Zero(space.getDim())) {}

Sampler::Sampler(const EuclideanSpace &space, const State &start, const State &goal, const double &best_cost)
    : dim_(space.getDim()),
      rand_(std::mt19937(std::random_device()())),
      dist_gauss_(std::normal_distribution<>(0.0, 1.0)),
      dist_unit_(std::uniform_real_distribution<>(0.0, 1.0)),
      dist_space_(generateSpaceDistribution(space)),
      min_cost_(goal.distanceFrom(start)),
      best_cost_(best_cost),
      rotate_mat_(calcRotationToWorldFlame(start, goal)),
      center_state_([&] {
        auto center_v = ((start + goal) / 2).vals;
        center_v.push_back(0.0);
        return Eigen::Map<Eigen::VectorXd>(&center_v[0], center_v.size());
      }()) {}

Sampler::~Sampler() {}

void Sampler::applyStartAndGoal(const State &start, const State &goal) {
  min_cost_ = goal.distanceFrom(start);
  rotate_mat_ = calcRotationToWorldFlame(start, goal);

  auto center_v = ((start + goal) / 2).vals;
  center_v.push_back(0.0);
  center_state_ = Eigen::Map<Eigen::VectorXd>(&center_v[0], center_v.size());
}

void Sampler::setBestCost(const double &best_cost) { best_cost_ = best_cost; }

double Sampler::getUniformUnitRandomVal() { return dist_unit_(rand_); }

State Sampler::run(const Sampler::Mode &mode) {
  State random_state(dim_);
  switch (mode) {
    case Mode::WholeArea: {
      for (size_t i = 0; i < dim_; i++) {
        random_state.vals[i] = dist_space_[i](rand_);
      }
      break;
    }
    case Mode::HeuristicDomain: {
      // definition of diagonal element
      auto diag_val = std::sqrt(std::pow(best_cost_, 2) - std::pow(min_cost_, 2)) / 2.0;
      auto diag_v = std::vector<double>(dim_ + 1, diag_val);
      diag_v[0] = best_cost_ / 2.0;

      // random sampling on unit n-ball
      auto x_ball_v = sampleUnitNBall(dim_).vals;
      x_ball_v.push_back(0.0);

      // trans sampling pt
      auto rand = rotate_mat_ * Eigen::Map<Eigen::VectorXd>(&*diag_v.begin(), diag_v.size()).asDiagonal() *
                      Eigen::Map<Eigen::VectorXd>(&*x_ball_v.begin(), x_ball_v.size()) +
                  center_state_;

      auto row_i = 0;
      for (auto &val : random_state.vals) {
        val = rand(row_i, 0);
        row_i++;
      }
    } break;
  }

  return random_state;
}

std::vector<std::uniform_real_distribution<>> Sampler::generateSpaceDistribution(const EuclideanSpace &space) const {
  std::vector<std::uniform_real_distribution<double>> rand_restrictions;
  rand_restrictions.reserve(space.getDim());
  for (size_t di = 1; di <= space.getDim(); di++) {
    rand_restrictions.emplace_back(space.getBound(di).low, space.getBound(di).high);
  }
  return rand_restrictions;
}

Eigen::MatrixXd Sampler::calcRotationToWorldFlame(const State &start, const State &goal) const {
  if (start.getDim() != goal.getDim() || start.getDim() < 2) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "State dimension is invalid");
  } else if (goal.distanceFrom(start) == 0) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "Start and goal are same state");
  }

  auto a1_state = (goal - start) / goal.distanceFrom(start);
  auto a1_v = a1_state.vals;
  a1_v.push_back(0.0);

  auto M = Eigen::Map<Eigen::VectorXd>(&*a1_v.begin(), a1_v.size()) * Eigen::MatrixXd::Identity(1, a1_v.size());
  auto svd = Eigen::JacobiSVD<Eigen::MatrixXd>(M, Eigen::ComputeFullU | Eigen::ComputeFullV);

  auto diag_v = std::vector<double>(a1_v.size(), 1.0);
  diag_v[diag_v.size() - 1] = svd.matrixV().determinant();
  diag_v[diag_v.size() - 2] = svd.matrixU().determinant();

  return svd.matrixU() * Eigen::Map<Eigen::VectorXd>(&*diag_v.begin(), diag_v.size()).asDiagonal() *
         svd.matrixV().transpose();
}

State Sampler::sampleUnitNBall(const uint32_t &dim) {
  if (dim == 0) {
    throw std::invalid_argument("[" + std::string(__PRETTY_FUNCTION__) + "] " + "Can not sample zero-dimension ball");
  }

  State x(dim);
  while (true) {
    for (auto &v : x.vals) {
      v = dist_gauss_(rand_);
    }
    auto r = x.norm();
    if (r != 0.0) {
      x = x / r;
      break;
    }
  }

  return x * std::pow(dist_unit_(rand_), 1.0 / dim);
}
}  // namespace planner
