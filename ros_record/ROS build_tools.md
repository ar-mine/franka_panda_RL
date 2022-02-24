# [A universal build tool for ROS](https://design.ros2.org/articles/build_tool.html)

# [Catkin_make and ament_make](http://design.ros2.org/articles/ament.html)
## Design concept
When using ROS, we need to build multi packages to achieve the goal. While it is complex to handle with the dependency among interdependent packages. Without build tools, programmer needs to be careful to build these packages while following the order recorded on the documents. ROS provides a set of tools to mitigate the complexity and make the process automatically.
## Basic parts
- a build system (e.g. CMake, Python setuptools) to configure, build, and install a single package
- a tool to invoke the build of individual packages in their topological order
- `package.xml` -> meta information about the packages to determine their dependencies and their build type
## [ament_cmake user documentation](https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html)
# Colcon
1. [Colcon simple tutorial](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)
# [Cmake tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)

# [REP-149](https://www.ros.org/reps/rep-0149.html)