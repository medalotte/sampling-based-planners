# sampling-based-planners
Pure C++ implementation of RRT, RRT*, and Informed-RRT* as a shared library which are incremental sampling-based path planning methods, and it supports any dimensions

#### TODO
- Implement Informed-RRT* (Sorry, it has not been implemented yet)
- Implement `examples/path-planner-3D`
    - abopt python API (Python.h) and use matplotlib in order to plot 3D graph

## Feature
- Provided as a shared library usable in **C++14** or higher
- You can execute at **any dimensions** without recompiling the shared library
- **Pure C++ implementation**
    - Implemented without dependent libraries such as Boost, Eigen, OpenCV, and so on

## Requirement
The following software packages are required for building the shared library
- A C++ compiler with **C++14** or higher support
- CMake **3.0** or higher

If you would like to compile the example programs, add the following:
- OpenCV **3.0** or higher

## Build
The shared library (**libplanner.so**) can be build with following commands

``` sh
$ cd <specific directory(no problem anywhere)>
$ git clone git@github.com:kyk0910/sampling-based-planners.git
$ cd <specific directory>/sampling-based-planners/lib
$ mkdir build && cd build
$ cmake ..
$ make
```

### Example programs
The example program can be run with following commands after build the shared library

``` sh
$ cd <specific directory>/sampling-based-planners
$ git submodule update --init
$ cd <specific directory>/sampling-based-planners/examples/<example program directory>
$ mkdir build && cd build
$ cmake ..
$ make
```

#### Example1. path-planning-2D
Execute path planning on two-dimensional space

##### Pattern1. Constraint besed on image
<div style="text-align: center;">
    <img src="assets/result_2D_img.png" alt="result_2D_img.png" width=50%>
</div>

##### Pattern2. Constraint besed on set of circle
<div style="text-align: center;">
    <img src="assets/result_2D_circle.png" alt="result_2D_circle.png" width=50%>
</div>

## Usage
Please read source code of expample programs.

## Refarences
[Steven M. LaValle, "Rapidly-exploring random trees: A new tool for path planning," Technical Report. Computer Science Department, Iowa State University (TR 98–11).](http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf)

[S. Karaman and E. Frazzoli, "Incremental Sampling-based Algorithms for Optimal Motion Planning," arXiv:1005.0416, May. 2010.](https://arxiv.org/pdf/1005.0416.pdf)

## License
MIT
