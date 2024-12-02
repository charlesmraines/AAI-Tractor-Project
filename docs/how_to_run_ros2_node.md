# Running a ROS 2 Humble Node

This guide will walk you through the steps of running a ROS 2 Humble node. It covers setting up your environment, creating a basic package, building it, and running the node.

## Prerequisites

- Install ROS 2 Humble Hawksbill
- Install dependencies like `colcon` (ROS 2 build tool)
- A workspace setup with ROS 2 packages

### 1. Set Up the ROS 2 Environment

Before creating and running a ROS 2 node, you need to set up the environment.

1. Source the ROS 2 Humble environment:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
If you are using the Jetson Orin Nano on the Tractor, you do not need to perform this step.

### 2. Building and Sourcing the ROS 2 Workspace

1. Navigate to the `~/AAI-TRACTOR_PROJECT/tractor_ws` directory:
    ```bash
    cd ~/tractor_ws
    ```

2. Build the workspace:
    ```bash
    colcon build
    ```

3. Source the workspace:
    ```bash
    source install/setup.bash
    ```

### 3. Run the Node

To run a node in ROS2, simply use the following format:

```bash
ros2 run my_package my_node
```

For example, to run the p_controller node, the command would be:

```bash
ros2 run tractor_control p_controller_node
```

### 4. Conclusion

You should now be able to run a ros2 node on the AAI New Holland tractor.