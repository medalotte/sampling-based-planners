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

#ifndef PATH_PLANNING_SRC_MAIN_H_
#define PATH_PLANNING_SRC_MAIN_H_

#include <memory>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <YAMLHelper.hpp>

#include <planner.h>

class RRTParam : public cvlib::YAMLHelper::Param {
public:
    int    max_sampling_num;
    double goal_sampling_rate;
    double expand_dist;
    void read(const cv::FileNode& node) override {
        max_sampling_num   = (int)node["max_sampling_num"];
        goal_sampling_rate = (double)node["goal_sampling_rate"];
        expand_dist        = (double)node["expand_dist"];
    }
};

class RRTStarParam : public cvlib::YAMLHelper::Param {
public:
    int    max_sampling_num;
    double goal_sampling_rate;
    double expand_dist;
    double R;
    void read(const cv::FileNode& node) override {
        max_sampling_num   = (int)node["max_sampling_num"];
        goal_sampling_rate = (double)node["goal_sampling_rate"];
        expand_dist        = (double)node["expand_dist"];
        R                  = (double)node["R"];
    }
};

class StateParam : public cvlib::YAMLHelper::Param {
public:
    double x;
    double y;
    void read(const cv::FileNode& node) override {
        x = (double)node["x"];
        y = (double)node["y"];
    }
};

class ObstacleParam : public cvlib::YAMLHelper::Param {
public:
    double x;
    double y;
    double radius;
    void read(const cv::FileNode& node) override {
        x      = (double)node["x"];
        y      = (double)node["y"];
        radius = (double)node["radius"];
    }
};

#endif /* PATH_PLANNING_SRC_MAIN_H_ */
