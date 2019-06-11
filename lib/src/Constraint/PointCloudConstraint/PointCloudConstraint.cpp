/**
 *  MIT License
 *
 *  Copyright (c) 2019 Yuya Kudo
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#include <Constraint/PointCloudConstraint/PointCloudConstraint.h>

namespace planner {
    PointCloudConstraint::Hypersphere::Hypersphere():
        state_(State()), radius_(0) {
    }

    PointCloudConstraint::Hypersphere::Hypersphere(const State& state, const double& radius) :
        state_(state), radius_(std::abs(radius)) {
    }

    void PointCloudConstraint::Hypersphere::set(const State& state, const double& radius) {
        state_ = state;
        radius_ = std::abs(radius);
    }

    void PointCloudConstraint::Hypersphere::setRadius(const double& radius)  {
        radius_ = std::abs(radius);
    }

    void PointCloudConstraint::Hypersphere::setState(const State& state) {
        state_ = state;
    }

    State PointCloudConstraint::Hypersphere::getState() const {
        return state_;
    }

    double PointCloudConstraint::Hypersphere::getRadius() const {
        return radius_;
    }

    PointCloudConstraint::PointCloudConstraint(const EuclideanSpace& space) :
        base::ConstraintBase(space) {
    }

    PointCloudConstraint::PointCloudConstraint(const EuclideanSpace&           space,
                                               const std::vector<Hypersphere>& constraint) :
        base::ConstraintBase(space) {
        set(constraint);
    }

    PointCloudConstraint::~PointCloudConstraint() {
    }

    void PointCloudConstraint::set(const std::vector<Hypersphere>& constraint) {
        for(const auto& data : constraint) {
            if(getDim() != data.getState().getDim()) {
                throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                         "State dimension is invalid");
            }
        }

        constraint_ = constraint;
    }

    const std::vector<PointCloudConstraint::Hypersphere>& PointCloudConstraint::getRef() const {
        return constraint_;
    }

    ConstraintType PointCloudConstraint::checkConstraintType(const State& state) const {
        if(getDim() != state.getDim()) {
            throw std::runtime_error("[" + std::string(__PRETTY_FUNCTION__) + "] " +
                                     "State dimension is invalid");
        }

        for(const auto& data : constraint_) {
            if(state.distanceFrom(data.getState()) < data.getRadius()) {
                return ConstraintType::NOENTRY;
            }
        }

        return ConstraintType::ENTAERABLE;
    }
}

