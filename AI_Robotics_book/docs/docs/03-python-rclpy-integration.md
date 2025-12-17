---
id: 03-python-rclpy-integration
title: "Chapter 3: Building Applications with rclpy"
sidebar_label: "3. Building with rclpy"
---

## Chapter 3: Building Applications with rclpy

**Objective**: Master the creation of custom ROS 2 packages, nodes, and launch files using the `rclpy` (ROS Client Library for Python) library.

### 3.1 The ROS 2 Workspace and Package Structure

Before writing code, it's crucial to understand how ROS 2 organizes projects. All your code should live within a **workspace**, which is essentially a directory containing your ROS 2 packages. A typical workspace has the following structure:

-   `ros2_ws/` (your workspace root)
    -   `src/`: This is where you place the source code for all your ROS 2 packages.
    -   `build/`: An intermediate directory where `colcon` (the ROS 2 build tool) processes your packages.
    -   `install/`: The final destination for your built packages, including executables and setup files.
    -   `log/`: Contains logs from the build process.

#### Creating a Python Application Package
A ROS 2 **Package** is a directory containing your nodes, launch files, interface definitions, and a manifest file. It's the fundamental unit of software organization in ROS. For a Python-based application, we create an `ament_python` package.

From within your `ros2_ws/src` directory, run:
```bash
ros2 pkg create --build-type ament_python --node-name battery_monitor my_robot_app
```
This creates a package named `my_robot_app` with a sample node named `battery_monitor`. Let's examine the key generated files:

-   `package.xml`: The package manifest. It contains meta-information like the package name, version, author, and, most importantly, its dependencies. For example, you must list which other ROS 2 packages you `depend` on.
-   `setup.py`: The Python setup script. Its primary role is to define the `entry_points` for your nodes. This is how ROS 2 knows that a name like `battery_monitor` corresponds to a function in a specific Python file, making it an executable.
-   `setup.cfg`: A configuration file for `setup.py`.

#### Creating a Custom Interface Package
A core principle of ROS is **separating interfaces from implementation**. Your custom message, service, and action definitions should live in their own dedicated package. This package must be a `ament_cmake` package, even if you are only using it with Python nodes.

From `ros2_ws/src`, create a second package for our interfaces:
```bash
ros2 pkg create --build-type ament_cmake my_robot_interfaces
```
This package requires special configuration:
1.  **In `package.xml`**, you must add dependencies that give your package the ability to generate code from interface files:
    ```xml
    <buildtool_depend>rosidl_default_generators</buildtool_depend>
    <depend>rosidl_default_runtime</depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```
2.  **In `CMakeLists.txt`**, you must tell the build system to find the generator and which files to turn into code:
    ```cmake
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/BatteryState.msg"
    )
    ```
3.  **In `my_robot_app/package.xml`**, you must add a dependency on your new interface package:
    ```xml
    <depend>my_robot_interfaces</depend>
    ```

Now, create a `msg` directory inside `my_robot_interfaces` and add the following file:

**`my_robot_interfaces/msg/BatteryState.msg`**
```
std_msgs/Header header
float32 voltage
float32 percentage
bool is_charging
```

#### Building the Workspace
To build your packages, navigate to the root of your workspace (`ros2_ws`) and run:
```bash
colcon build
```
`colcon` will automatically discover your packages and build them in the correct order, building `my_robot_interfaces` first. After a successful build, you must source the new setup file in the `install` directory to make your new packages and messages available in your terminal:
```bash
source install/setup.bash
```

### 3.2 Writing a Custom Node with `rclpy`

Let's rewrite our `battery_monitor` node to use our new custom message. Using a class that inherits from `rclpy.node.Node` is the standard way to structure a node, as it helps manage state and organize communication callbacks.

```python
# In my_robot_app/my_robot_app/battery_monitor.py
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import BatteryState # Import our custom message

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        # Note the change from Float32 to BatteryState
        self.publisher_ = self.create_publisher(BatteryState, 'battery_level', 10)
        self.timer = self.create_timer(1.0, self.publish_battery_level)
        self.battery_level_ = 100.0
        self.get_logger().info('Battery Monitor node started.')

    def publish_battery_level(self):
        # Create an instance of our custom message
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = 12.5 - ( (100.0 - self.battery_level_) / 100.0 * 2.5 )
        msg.percentage = self.battery_level_
        msg.is_charging = False
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing battery level: {self.battery_level_:.2f}%')
        self.battery_level_ -= 0.1
        if self.battery_level_ < 0:
            self.battery_level_ = 100.0
```

### 3.3 Advanced Parameter Management
Hardcoding values like the battery drain rate is bad practice. Parameters make your nodes reusable and configurable.

*[Image: A diagram showing the parameter override hierarchy. A value from a YAML file is overridden by a value in a launch file, which is in turn overridden by a value from the command line (`ros2 param set`).]*

Let's add a parameter for the drain rate and a dynamic callback to update it on the fly.

```python
# In my_robot_app/my_robot_app/battery_monitor.py, updated __init__
def __init__(self):
    super().__init__('battery_monitor')
    
    # Declare the parameter and its default value
    self.declare_parameter('drain_rate', 0.1)
    
    # Get the initial value
    self.drain_rate_ = self.get_parameter('drain_rate').get_parameter_value().double_value
    
    # Add a callback for when parameters are changed
    self.add_on_set_parameters_callback(self.parameter_callback)

    # ... (rest of __init__)

# Add the callback method to the class
def parameter_callback(self, params):
    for param in params:
        if param.name == 'drain_rate':
            self.drain_rate_ = param.value
            self.get_logger().info(f'Drain rate updated to: {self.drain_rate_}')
    return rclpy.parameter.Parameter.SetParametersResult(successful=True)
```
Now, you can change the drain rate while the node is running with the command:
`ros2 param set /battery_monitor drain_rate 0.5`

#### Using YAML for Parameters
For complex systems, you should store parameters in a YAML file.

**`my_robot_app/config/params.yaml`**
```yaml
battery_monitor:
  ros__parameters:
    drain_rate: 0.2
```

### 3.4 Python Launch Files
Launch files are Python scripts that allow you to start and configure a complex system of multiple nodes at once. They are the standard way to run a ROS 2 application.

Let's create a launch file that starts our battery monitor and loads its parameters from our YAML file.

```python
# In my_robot_app/launch/my_launch.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to our parameter file
    config = os.path.join(
        get_package_share_directory('my_robot_app'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_app',
            executable='battery_monitor',
            name='battery_monitor',
            parameters=[config] # Pass the config file to the node
        ),
        # You could add other nodes here
    ])
```
You would then need to update `setup.py` to tell ROS 2 where to find this launch file. Finally, you can run everything with a single command:
`ros2 launch my_robot_app my_launch.launch.py`

This chapter provides the foundational skills for creating complete, configurable, and reusable ROS 2 applications with Python.