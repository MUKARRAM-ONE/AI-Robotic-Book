---
id: 04-urdf-for-humanoids
title: "Chapter 4: Describing Robots with URDF"
sidebar_label: "4. URDF for Humanoids"
---

## Chapter 4: Describing Robots with URDF

**Objective**: Learn to describe a robot's physical structure for simulation and visualization using the Unified Robot Description Format (URDF) and its powerful extension, Xacro.

### 4.1 Introduction to URDF

The **Unified Robot Description Format (URDF)** is an XML-based file format for representing a robot model. In ROS, URDF is the fundamental standard for describing the physical properties of a robot. This description is not just for looks; it is the lynchpin for almost all high-level robotics tasks, including:
- **Simulation**: A physics engine like Gazebo uses the URDF to understand how the robot will interact with its environment.
- **Visualization**: Tools like RViz2 use the URDF to display a 3D model of the robot and its sensor data.
- **Kinematics & Planning**: Motion planning libraries use the URDF to calculate valid joint positions and plan collision-free paths.

At its core, a URDF describes a robot as a **tree of links and joints**. 
-   **Links** are the rigid body parts of the robot.
-   **Joints** connect the links and define how they can move relative to one another.
Every robot model must have a single root link, often called `base_link`, from which all other links branch out. This tree structure is fundamental, and URDF does not allow for closed loops or cycles in the kinematic chain.

### 4.2 The `<link>` Tag in Detail

A link represents a single rigid body part. The `<link>` tag has three important children: `<visual>`, `<collision>`, and `<inertial>`. It is critical to understand their distinct roles.

#### `<visual>`: What the Robot Looks Like
This tag describes the appearance of the link. It is used *only* for visualization in tools like RViz2 and has no impact on physics simulation.
- **`<geometry>`**: Can be a simple primitive (`<box>`, `<cylinder>`, `<sphere>`) or, more commonly, a 3D mesh file (`<mesh filename="package://my_robot_description/meshes/torso.stl"/>`). It's good practice to use low-polygon meshes for visualization to maintain performance.
- **`<material>`**: Defines the color and texture of the link.

#### `<collision>`: What the Robot Interacts With
This tag defines the collision geometry of the link. It is used *only* by the physics engine to calculate collisions. To improve simulation performance, the collision geometry is often much simpler than the visual geometry. For example, a visually complex robot arm could be represented by a series of simple cylinders for collision purposes.

#### `<inertial>`: How the Robot Behaves
This tag defines the dynamic properties of the link: its mass and inertia tensor. These values are critical for a realistic physics simulation.
- **`<mass value="..."/>`**: The mass of the link in kilograms.
- **`<inertia ixx="..." iyy="..." ... />`**: The 6-element inertia tensor, which describes the link's resistance to rotation around its different axes. While complex to calculate by hand, 3D modeling software like Blender or SolidWorks can often compute these values from a 3D model.

### 4.3 The `<joint>` Tag in Detail

A joint connects a `parent` link to a `child` link, defining their relative motion.

- **`<parent link="..."/>`** & **`<child link="..."/>`**: Defines the two links the joint connects.
- **`<origin xyz="..." rpy="..."/>`**: Specifies the pose of the child link's origin relative to the parent link's origin. `xyz` is the position offset, and `rpy` is the orientation offset in roll, pitch, and yaw.
- **`<axis xyz="..."/>`**: For moving joints, this defines the axis of rotation or translation in the joint's reference frame.
- **`<limit lower="..." upper="..." effort="..." velocity="..."/>`**: For `revolute` and `prismatic` joints, this tag defines the motion limits.

#### Common Joint Types:
| Type | Description | Example Use Case |
| :--- | :--- | :--- |
| `revolute` | Rotates around a single axis with defined limits. | Elbow, knee, or shoulder joint. |
| `continuous` | Rotates around a single axis with no limits. | A powered wheel on a mobile robot. |
| `prismatic` | Slides along a single axis with defined limits. | A linear actuator or a gripper's finger. |
| `fixed` | A rigid, unmoving connection. | Mounting a sensor or a camera rigidly to a robot's body. |
| `floating` | Allows motion in all 6 degrees of freedom. | Connecting the root link of a mobile robot to the world frame. |
| `planar` | Allows motion in a 2D plane (translation in x/y, rotation in z). | A robot moving on a flat surface. |

### 4.4 Improving URDFs with Xacro
Writing raw URDF is verbose and repetitive. **Xacro** (XML Macros) is the standard tool for making URDFs clean, modular, and maintainable. It is a pre-processor that runs before the URDF is parsed.

#### Key Xacro Features:
-   **Properties**: Define constants to avoid "magic numbers".
    ```xml
    <xacro:property name="wheel_radius" value="0.1"/>
    ```
-   **Macros**: Create reusable templates for common elements, like a robot arm or a sensor.
    ```xml
    <xacro:macro name="default_inertial" params="mass">
      <inertial>
        <mass value="${mass}" />
        <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
      </inertial>
    </xacro:macro>
    ```
-   **Includes**: Split your URDF into multiple files for better organization.
    ```xml
    <xacro:include filename="$(find my_robot_description)/urdf/arm.xacro" />
    ```

#### Rewriting the Humanoid with Xacro
Here is a much cleaner version of the simple humanoid, using properties and macros.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Define Constants -->
  <xacro:property name="torso_x" value="0.2"/>
  <xacro:property name="torso_y" value="0.3"/>
  <xacro:property name="torso_z" value="0.5"/>
  <xacro:property name="head_radius" value="0.1"/>

  <!-- Macro for a simple box link -->
  <xacro:macro name="box_link" params="name mass size">
    <link name="${name}">
      <visual>
        <geometry>
          <box size="${size}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <box size="${size}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Instantiate Links using Macros -->
  <xacro:box_link name="torso" mass="10" size="${torso_x} ${torso_y} ${torso_z}"/>
  
  <link name="head">
    <!-- ... (head definition) ... -->
  </link>

  <!-- Joints -->
  <joint name="neck_joint" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 ${torso_z/2 + head_radius/2}" rpy="0 0 0"/>
  </joint>
  
  <!-- ... (rest of robot) ... -->
</robot>
```
Notice how properties (`${torso_z}`) can be used in mathematical expressions, making the model parametric and much easier to adjust.

### 4.5 Visualization in RViz2
The primary reason to create a URDF is to see your robot. **RViz2** is ROS 2's powerful 3D visualization tool. To view your URDF, you need two key nodes:
-   **`robot_state_publisher`**: This node reads your URDF from the `/robot_description` topic and listens to the `/joint_states` topic. Based on the joint angles it receives, it calculates the 3D pose of every link in the robot and publishes these as `tf2` transforms.
-   **`joint_state_publisher_gui`**: This handy tool provides a simple GUI with sliders for every non-fixed joint in your URDF. By moving the sliders, you publish messages to the `/joint_states` topic, which `robot_state_publisher` then uses.

#### Launching the Visualization
The standard way to view a URDF is with a launch file that starts everything you need.

```python
# A typical URDF visualization launch file
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('my_robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'my_robot.urdf.xacro')
    
    # Use xacro to process the file
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_path, 'config', 'view_robot.rviz')]
        )
    ])
```
Running this launch file will:
1.  Convert your `.xacro` file into a URDF string.
2.  Start `robot_state_publisher` with your URDF.
3.  Start `joint_state_publisher_gui` so you can move the joints.
4.  Start RViz2, pre-configured to display your robot.

![Example of a robot model visualized in RViz2](/img/rivz_model.jpg)
*Example of a robot model visualized in RViz2 (Courtesy of Fictionlab under CC BY-NC-SA).*

### 4.5 Building a Humanoid Robot URDF

Humanoid robots are among the most complex robots to model accurately, as they must capture the articulated structure of a human body while maintaining the kinematic constraints of real motors and joints.

#### Key Considerations for Humanoid URDFs:

1. **Symmetry**: Humanoids typically have left and right limbs. Use Xacro macros and parameters to avoid code duplication.
2. **Hierarchical Structure**: Organize your URDF by body parts (head, torso, left_arm, right_arm, legs).
3. **Accurate Sensor Mounting**: Cameras, IMUs, and range sensors must be precisely located and oriented relative to the body.
4. **Center of Mass**: For bipedal balance, the center of mass location is critical. It must lie within the polygon of support (the area between the robot's feet).

#### Example Structure:
```
base_link (torso center)
├── head_link
├── left_shoulder_link
│   └── left_upper_arm_link
│       └── left_forearm_link
│           └── left_hand_link
├── right_shoulder_link
│   └── right_upper_arm_link
│       └── right_forearm_link
│           └── right_hand_link
├── left_hip_link
│   └── left_thigh_link
│       └── left_calf_link
│           └── left_foot_link
└── right_hip_link
    └── right_thigh_link
        └── right_calf_link
            └── right_foot_link
```

### 4.6 Validating and Debugging Your URDF

Even with the best of intentions, URDF files often contain errors. ROS provides several tools for validation:

- **`urdf_parser`**: A command-line tool that catches malformed XML and semantic errors.
  ```bash
  check_urdf my_robot.urdf
  ```
- **Visual Inspection**: After loading in RViz2, visually inspect that the robot's appearance matches your expectations. Look for:
  - Incorrect link orientations or positions.
  - Missing or misaligned sensors.
  - Collision geometry that doesn't match the visual geometry.
- **`urdf_to_graphviz`**: Generates a graph visualization of your robot's structure.
  ```bash
  urdf_to_graphviz my_robot.urdf
  dot -Tsvg my_robot.svg -o my_robot.svg
  ```

### 4.7 Integration with Physics Engines

When you use your URDF in Gazebo, additional tags become important:

- **`<gazebo>` tags**: Provide Gazebo-specific parameters for each link or joint.
  ```xml
  <gazebo reference="left_wheel">
    <mu1>0.5</mu1>
    <mu2>0.5</mu2>
  </gazebo>
  ```
- **`<plugin>` tags**: Allow Gazebo to run specialized code for that link (e.g., a camera sensor plugin that simulates image capture).

## Summary

The URDF is the backbone of your robot model. It describes the physical structure, the kinematic tree, and provides the parameters needed for simulation and visualization. Xacro makes URDFs practical for complex robots by introducing modularity and reusability. Mastering URDF is essential for any roboticist, as it influences every downstream task from simulation to motion planning to perception.