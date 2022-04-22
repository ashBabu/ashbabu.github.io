---
layout: post
title: Matplotlib in C++ with CMake
date: 2021-11-08 11:12:00-0400
description: How to make a matplotlib-cpp work with CMake
categories: Programming
---

[Matplotlib-cpp](https://github.com/lava/matplotlib-cpp) is a header only library that exposes the functionalities of python matplotlib library to C++. The following example plots a figure as shown below.

    {% highlight c++ linenos %}
    #include "matplotlibcpp.h"
    namespace plt = matplotlibcpp;
    int main()
    {
        plt::plot({1,3,2,4});
        plt::show();
    }
    {% endhighlight %}

{% include figure.html path="assets/img/programming/matplotlib-cpp.png" class="img-fluid rounded z-depth-1" zoomable=true %}
<br/>
The command line code to build the program is as follows

```g++ minimal.cpp -o minimal -std=c++11 -I/usr/include/python2.7 -lpython2.7```

To make the same work with `CMake`, the required lines in `CMakeLists.txt` are

```
cmake_minimum_required (VERSION 3.0)
project(minimal)

find_package(PythonLibs REQUIRED)
find_package(PkgConfig REQUIRED)

include_directories(
    ${PYTHON2_INCLUDE_DIRS}
    "/usr/include/python2.7"
)

add_executable(
    ${PROJECT_NAME}
    minimal.cpp
)

target_link_libraries(
    ${PROJECT_NAME}
    python2.7
)
```


