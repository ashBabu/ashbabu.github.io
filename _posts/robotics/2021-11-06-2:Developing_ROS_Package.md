---
layout: post
title: "Part2: Developing a ROS C++ package"
date: 2021-12-07 11:30:00-0400
description: How to make a ROS package
categories: Robotics Programming
---
### A closer look at *CMakeLists.txt* and *package.xml*


We looked at creating a catkin package in [Part1](/blog/2021/1-Developing_ROS_Package/). A catkin package is characterised by a `CMakeLists.txt` and a `package.xml`. Lets take a deeper look into the important parts of these. Only the main parts of `CMakeLists.txt` is shown below and this is in no way a complete tutorial to CMake. By default, `catkin_create_pkg` generates a long `CMakeLists.txt` with lots of comments for users to read and understand. The external settings that are used to create the package are listed under the `find_package()` tag. An excellent explanation to what `catkin_package()` is provided in [here](https://answers.ros.org/question/58498/what-is-the-purpose-of-catkin_depends/). 

```
cmake_minimum_required(VERSION 3.0.2)
project(my_test_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES my_test_pkg
  CATKIN_DEPENDS moveit_core roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/my_test_pkg.cpp)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
 )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(
        test_one
        test/test_one.test
        test/test_one.cpp
    )
    target_link_libraries(
        test_one
        ${catkin_LIBRARIES}
    )
    add_dependencies(
        test_one
        ${catkin_EXPORTED_TARGETS}
    )

    catkin_add_gtest(
        test_two
        test/test_two.cpp
        src/hello.cpp
        src/add.cpp
    )
    target_link_libraries(
        test_two
        ${catkin_LIBRARIES}
    )
```

The `include_directories()` contain the header files of the packages that you use in your project. If the project uses [OpenCV](https://opencv.org/) as well, then this has to be added as
```
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
  ) 
```
Lets say we want to create two header files like `hello.h` and `add.h`, it would be a good practice to organize it as follows. 

```
my_test_pkg
        |_ CMakeLists.txt
        |_ package.xml
        |_ src/
        |_ include/
              |_ my_test_pkg/
                    |_ greet/
                        |_ hello.h
                    |_ math/
                        |_ add.h
```
The header `hello.h` would be something like

```
#ifndef _HELLO_H_
#define _HELLO_H_

#include<iostream>
#include<string>

namespace greeting
{
class Hello
{
    public:
        Hello(); // constructor
        ~Hello();  // destructor
        void printHello(const std::string & str):
    protected:
        // protected attributes

    private:
        // private attributes
};
}
#endif _HELLO_H_
```

Then similarly in the `src`, create two folders `greet` and `math` with the corresponding implementations (for example `hello.cpp` and `add.cpp`). The headers can then be used as `#include <my_test_pkg/greet/hello.h>` and `#include <my_test_pkg/math/add.h>`.

The `hello.cpp` would be like

```
#include <my_test_pkg/greet/hello.h>

using namespace greeting;

Hello::Hello()
{
    std::cout<<"Hello Constructor Invoked"<< std::endl;
}
Hello::~Hello()
{
    std::cout<<"Hello Destructor Invoked"<< std::endl;
}
void Hello::printHello(const std::string & str)
{
    std::cout<<" The string to print is " << str << std::endl;
}
```
As you know that a C++ program allows only one `main()` function, we could take a look at how this is organised in the `src` directory.

```
my_test_pkg
        |_ CMakeLists.txt
        |_ package.xml
        |_ include/
        |_ src/
            |_ greet/
                |_ hello.cpp
            |_ math/
                |_ add.cpp
            |_ main.cpp
```

The `main.cpp` will look something like

```
#include <my_test_pkg/greet/hello.h>
#include<my_test_pkg/math/add.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_test_node");  // initiating a ros node
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Rate loop_rate(5);
    int N[100] = {1, 2, 3 ....};
    std::string myName = "JonDoe";

    while (ros::ok())
    {
        // call your functions: as an example
        printHello(myName);
        addNumbers(N)
        loop_rate.sleep();
    }

    spinner.stop();
    return 0;
//    // Wait for ROS threads to terminate
//    ros::waitForShutdown();
}
```
A little bit of ROS specific stuffs are involved but it should be quite straight forward.

The `add_executable()` is where the whole executables are listed. In this particular case, it would be 
```
`add_executable( 
    src/hello.cpp
    src/add.cpp
    src/main.cpp
    )`
```
As the name goes, the  `add_dependencies()` help in adding the dependencies which is an essential part of your package. And finally the libraries are to be linked to your executable which is done using the 
```
target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
```

The `package.xml` should have the `build_depend` and `exec_depend` tags for all those listed under the `find_package()`. For a deeper understanding, have a look at [here](http://wiki.ros.org/catkin/package.xml). 

In no way, this article is a complete with respect to learning ROS CPP but I hope this would give a good enough start point. We will look at adding `gtest()` and `rostest()` in the next [article](/blog/2022/3-Developing_ROS_Package/).

