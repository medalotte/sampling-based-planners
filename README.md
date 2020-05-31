# sampling-based-planners
C++ implementation of RRT, RRT*, and Informed-RRT* using kd-tree for searching NN and NBHD nodes. Supports arbitrary dimensions and compiles as a shared library.

## Features
- Provided as a shared library usable in **C++14** or higher
- You can execute at **any dimensions** without recompiling the shared library
- To quickly search for NN and NBHD nodes, a node list consists of **kd-tree**.

## Requirements
The following software packages are required for building the shared library:
- A C++ compiler with **C++14** or higher support
- CMake **3.0** or higher
- Eigen **3.0** or higher

If you would like to compile the example programs, add the following:
- OpenCV **3.0** or higher

## Build
The shared library (**libplanner.so**) can be build with following commands

``` sh
$ git clone https://github.com/kyk0910/sampling-based-planners.git
$ cd sampling-based-planners/lib
$ mkdir build && cd build
$ cmake ..
$ make
```

The example program can be run with following commands after build the shared library

``` sh
$ cd <top of this repository>
$ git submodule update --init
$ cd examples/path-planning-2D
$ mkdir build && cd build
$ cmake ..
$ make
```

## Usage
### 1. Include header file and set alias optionally
``` c++
#include <planner.h>
namespace planner = pln
```

### 2. Define euclidean space
``` c++
// difinition of two-dimensional space
const int DIM = 2;
pln::EuclideanSpace space(DIM);

// definition of bounds of each dimension
std::vector<pln::Bound> bounds{pln::Bound(0, 100.0),
                               pln::Bound(0, 100.0)};

// set bounds to space
space.setBound(bounds);
```

### 3. Define constraints
#### i. Point cloud type
``` c++
// definition of obstacle (point cloud type)
std::vector<pln::PointCloudConstraint::Hypersphere> obstacles;
obstacles.emplace_back(pln::State(10.0, 20.0),  10.0);  // x : 10.0, y : 20.0, radius : 10.0
obstacles.emplace_back(pln::State(50.0, 70.0),  20.0);  // x : 50.0, y : 70.0, radius : 20.0
obstacles.emplace_back(pln::State(-10.0, 120.0), 30.0); // there is no probrem out of range

// definition of constraint using std::shared_ptr
auto constraint = std::make_shared<pln::PointCloudConstraint>(space, obstacles)
```

#### ii. Image type (use OpenCV for simplicity)
``` c++
// read image
auto world = cv::imread("./example.png", CV_8UC1);

// definition of constraint array
std::vector<pln::ConstraintType> map(world.cols * world.rows, pln::ConstraintType::ENTAERABLE);

for(int yi = 0; yi < world.rows; yi++) {
    for(int xi = 0; xi < world.cols; xi++) {
        if(world.data[xi + yi * world.cols] != 255) {
            map[xi + yi * world.cols] = pln::ConstraintType::NOENTRY;
        }
    }
}

std::vector<uint32_t> each_dim_size{(uint32_t)world.cols, (uint32_t)world.rows};

// definition of constraint using std::shared_ptr
auto constraint = std::make_shared<pln::GridConstraint>(space, map, each_dim_size);
```

### 4. Solve
``` c++
// definition of planner (you can set some parameters at optional argument)
// pln::RRT planner(DIM);
// pln::RRTStar planner(DIM);
pln::InformedRRTStar planner(DIM);

// set constraint
planner.setProblemDefinition(constraint);

// definition of start and goal state
pln::State start(5.0, 5.0);
pln::State goal(90.0, 90.0);

// solve
bool status = planner.solve(start, goal);
if(status) {
    auto& result = planner.getResult();
    for(const auto& r : result) {
        std::cout << r << std::endl;
    }
}
else {
    std::cout << "Could not find path" << std::endl;
}
```

## Example programs
### Example1. path-planning-2D
Execute path planning on two-dimensional space

#### Pattern1. Constraint using set of circle
<div style="text-align: center;">
    <img src="assets/result_2D_circle.png" alt="result_2D_circle.png">
    left: RRT, center: RRT*, right: Informed-RRT*
</div>


#### Pattern2. Constraint using image
<div style="text-align: center;">
    <img src="assets/result_2D_img.png" alt="result_2D_img.png">
    left: RRT, center: RRT*, right: Informed-RRT*
</div>

## References
[Steven M. LaValle, "Rapidly-exploring random trees: A new tool for path planning," Technical Report. Computer Science Department, Iowa State University (TR 98–11).](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf)

[S. Karaman and E. Frazzoli, "Incremental Sampling-based Algorithms for Optimal Motion Planning," arXiv:1005.0416, May. 2010.](https://arxiv.org/pdf/1005.0416.pdf)

[J. D. Gammell, S. S. Srinivasa, and T. D. Barfoot, “Informed RRT*: Optimal sampling-based path planning focused via direct sampling of an admissible ellipsoidal heuristic,” in 2014 IEEE/RSJ International Conference on Intelligent Robots and Systems, 2014, pp. 2997–3004.](https://ieeexplore.ieee.org/document/6942976/?arnumber=6942976)

## License
MIT
