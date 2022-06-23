---
layout: post
title: "Part3: Developing a ROS C++ package"
date: 2022-01-07 11:30:00-0400
description: How to make a ROS package
categories: Robotics Programming
---
### *gTest* and *rostest*


We looked at *CMakeLists.txt* and *package.xml* in [Part2](/blog/2021/2-Developing_ROS_Package/). The aim here is to add simple tests to verify the functionality of each of the functions or modules that is added to the project. This will also ensure that the package is not broken by adding something new to the catkin package. At the end of the `CMakeLists.txt`, add the following

```
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

`rostest` is basically the testing of your newly added function or module for compatibility with the ROS ecosystem. This is well documented [here](http://wiki.ros.org/rostest). Add a folder `test` in the same level as `include` (or `src`). `add_rostest_gtest()` is the function that enables testing which needs a name (`test_one`), a `.test` or `.launch` file and atleast one source file (`test_one.cpp`). The main difference between `rostest` and `gtest` is the lack of `.test` file. The `rostest` is invoked by using `rosrun package_name test_one.test` where the `test_one.test` would contain 
```
<launch>
    <test pkg="package_name" test-name="test_one" type="test_one" />
</launch>

```
The structure of `test_one.cpp` would be
```
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <my_test_package/greet/hello.h>

TEST(TESTSuite, my_test1)
{
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1); spinner.start();

    // define or find out a, b
    EXPECT_EQ(a, b);

    bool success = your_function();
    EXPECT_TRUE(success);
    std::cout<<" ::::::: "<<std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_ik_solutions");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

The functions `EXPECT_EQ` (for equality), `EXPECT_TRUE` (for boolean) are used to make sure if the test is run successfully or not. There are other functions as well which are well documented [here](https://google.github.io/googletest/reference/assertions.html). For diving deep, take a look at [Google Test](https://google.github.io/googletest/)