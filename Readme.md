# ros-node-interface

[![DOI](https://zenodo.org/badge/991307682.svg)](https://zenodo.org/badge/latestdoi/991307682)

A lightweight C++ interface template for managing ROS nodes with start/stop/shutdown control via ROS services.

This module provides a generic wrapper (`BaseRosInterface`) around any ROS-compatible module class, enabling it to be:
- Started and stopped using ROS service calls
- Managed with a clean threaded loop
- Extended for custom node types and behaviors

## Features

- Threaded execution loop (`loop_step`)
- Service-based control (`start`, `stop`, `shutdown`)
- Simple modular interface for defining new node types
- Example `TestModule` provided for demonstration

## Example Usage

```cpp
#include "ros-node-interface.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_node");

    auto my_module = std::make_unique<ros_node_interface::TestModule>(
        ros::NodeHandle(), "my_topic"
    );

    ros_node_interface::BaseRosInterface<ros_node_interface::TestModule> interface(
        std::move(my_module)
    );

    return 0;
}
````

## Dependencies

* ROS (tested with ROS Noetic)
* `std_srvs/Trigger`
* C++14 or newer
