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

#include "main.h"

int main() {
    const int DIM = 2;
    const std::string PARAM_YAML_DIRECTORY = "../data";
    const std::string PARAM_YAML_FILE_NAME = "param.yaml";
    const std::string PARAM_YAML_FILE_PATH = PARAM_YAML_DIRECTORY + "/" + PARAM_YAML_FILE_NAME;

    try {
        std::unique_ptr<planner::base::PlannerBase>    planner;
        std::shared_ptr<planner::base::ConstraintBase> constraint;

        RRTParam     rrt_param;
        RRTStarParam rrt_star_param;
        cvlib::YAMLHelper::readStruct(rrt_param,      PARAM_YAML_FILE_PATH, "RRT");
        cvlib::YAMLHelper::readStruct(rrt_star_param, PARAM_YAML_FILE_PATH, "RRTStar");

        StateParam world_size, start_state, goal_state;
        cvlib::YAMLHelper::readStruct(world_size,  PARAM_YAML_FILE_PATH, "WorldSize");
        cvlib::YAMLHelper::readStruct(start_state, PARAM_YAML_FILE_PATH, "StartState");
        cvlib::YAMLHelper::readStruct(goal_state,  PARAM_YAML_FILE_PATH, "GoalState");

        while(true) {
            bool end_flag = false;
            auto world    = cv::Mat(cv::Size(world_size.x, world_size.y), CV_8UC1, 0xFF);
            auto start    = planner::State(start_state.x, start_state.y);
            auto goal     = planner::State(goal_state.x,  goal_state.y);

            [&]() {
                while(true) {
                    char mode = '0';
                    std::cout << "[1] Please choose constraint type" << std::endl;
                    std::cout << "1. image" << std::endl;
                    std::cout << "2. set of circle" << std::endl;
                    std::cout << "q. quit" << std::endl << std::endl;
                    std::cout << "mode : ";
                    std::cin  >> mode;
                    std::cout << std::endl;

                    switch(mode) {
                        case '1': {
                            auto img_file_path = PARAM_YAML_DIRECTORY + "/" +
                                cvlib::YAMLHelper::read<std::string>(PARAM_YAML_FILE_PATH, "ConstraintImage");

                            world = cv::imread(img_file_path, CV_8UC1);

                            planner::EuclideanSpace space(DIM);
                            std::vector<planner::Bound> bounds{planner::Bound(0, world.cols),
                                                               planner::Bound(0, world.rows)};
                            space.setBound(bounds);

                            std::vector<uint32_t> each_dim_size{(uint32_t)world.cols,
                                                                (uint32_t)world.rows};

                            std::vector<planner::ConstraintType> constraint_map(world.cols * world.rows,
                                                                                planner::ConstraintType::ENTAERABLE);
                            for(int ri = 0; ri < world.rows; ri++) {
                                for(int ci = 0; ci < world.cols; ci++) {
                                    if(world.data[ci + ri * world.cols] != 255) {
                                        constraint_map[ci + ri * world.cols] = planner::ConstraintType::NOENTRY;
                                    }
                                }
                            }

                            constraint = std::make_shared<planner::SemanticSegmentConstraint>(space,
                                                                                              constraint_map,
                                                                                              each_dim_size);
                            return;
                        }
                        case '2': {
                            planner::EuclideanSpace space(DIM);
                            std::vector<planner::Bound> bounds{planner::Bound(0, world.cols),
                                                               planner::Bound(0, world.rows)};
                            space.setBound(bounds);

                            auto obstacle_num = cvlib::YAMLHelper::read<int>(PARAM_YAML_FILE_PATH, "Obstacles", "num");
                            std::vector<planner::PointCloudConstraint::Hypersphere> obstacles;
                            for(int i = 0; i < obstacle_num; i++) {
                                ObstacleParam obstacle_param;
                                cvlib::YAMLHelper::readStruct(obstacle_param, PARAM_YAML_FILE_PATH, "Obstacles", "Obstacle" + std::to_string(i));
                                obstacles.emplace_back(planner::State(obstacle_param.x, obstacle_param.y), obstacle_param.radius);
                            }

                            constraint = std::make_shared<planner::PointCloudConstraint>(space,
                                                                                         obstacles);

                            for(const auto& obstacle : static_cast<planner::PointCloudConstraint*>(constraint.get())->getRef()) {
                                cv::circle(world, cv::Point(obstacle.getState().vals[0], obstacle.getState().vals[1]),
                                           obstacle.getRadius(), 0, -1, CV_AA);
                            }

                            return;
                        }
                        case 'q': {
                            end_flag = true;
                            return;
                        }
                        default: {
                            break;
                        }
                    }
                }
            }();

            if(end_flag) {
                break;
            }

            [&]() {
                while(true) {
                    char mode = '0';
                    std::cout << "[2] Please choose method of path planning" << std::endl;
                    std::cout << "1. RRT" << std::endl;
                    std::cout << "2. RRT*" << std::endl;
                    std::cout << "q. quit" << std::endl << std::endl;
                    std::cout << "mode : ";
                    std::cin  >> mode;
                    std::cout << std::endl;

                    switch(mode) {
                        case '1': {
                            planner = std::make_unique<planner::RRT>(DIM,
                                                                     rrt_param.max_sampling_num,
                                                                     rrt_param.goal_sampling_rate,
                                                                     rrt_param.expand_dist);
                            return;
                        }
                        case '2': {
                            planner = std::make_unique<planner::RRTStar>(DIM,
                                                                         rrt_star_param.max_sampling_num,
                                                                         rrt_star_param.goal_sampling_rate,
                                                                         rrt_star_param.expand_dist,
                                                                         rrt_star_param.R);
                            return;
                        }
                        case 'q': {
                            end_flag = true;
                            return;
                        }
                        default: {
                            break;
                        }
                    }
                }
            }();

            if(end_flag) {
                break;
            }

            planner->setProblemDefinition(constraint);

            auto start_time = std::chrono::system_clock::now();
            bool status     = planner->solve(start, goal);
            auto end_time   = std::chrono::system_clock::now();

            if(status) {
                auto result = planner->getResult();

                cv::cvtColor(world, world, CV_GRAY2RGB);

                auto prev_node_pos = cv::Point(result[0].vals[0], result[0].vals[1]);
                for(const auto& r : result) {
                    std::cout << r.vals[0] << ", "
                              << r.vals[1]
                              << std::endl;

                    cv::circle(world, cv::Point(r.vals[0], r.vals[1]), 4.0, cv::Vec3b(128, 0, 255), 1.0, CV_AA);
                    cv::line(world, cv::Point(r.vals[0], r.vals[1]), prev_node_pos, cv::Vec3b(128, 255, 0), 2.0, CV_AA);
                    prev_node_pos = cv::Point(r.vals[0], r.vals[1]);
                }

                cv::putText(world, "Start", cv::Point(result.front().vals[0], result.front().vals[1]),
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200,0,0), 2, CV_AA);
                cv::putText(world, "Goal",  cv::Point(result.back().vals[0],  result.back().vals[1]),
                            cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(200,0,0), 2, CV_AA);

                cv::namedWindow("world", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
                cv::imshow("world", world);
                cv::waitKey(0);
                cv::destroyWindow("world");
                cv::imwrite("./result.png", world);
            }
            else {
                std::cout << "Could not find path" << std::endl;
            }

            std::cout << "elapsed time : "
                      << std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0
                      << "ms"
                      << std::endl
                      << std::endl;
        }
    }
    catch(const std::exception& e) {
        std::cout << e.what() << std::endl;
        exit(1);
    }
}
