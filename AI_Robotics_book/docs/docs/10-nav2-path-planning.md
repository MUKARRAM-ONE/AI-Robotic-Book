---
id: 10-nav2-path-planning
title: "Chapter 10: Nav2 Path Planning"
sidebar_label: "10. Nav2 Path Planning"
---

## Chapter 10: Nav2 Path Planning

**Objective**: Implement autonomous navigation for a humanoid robot using Nav2.

### 10.1 The Navigation2 (Nav2) Stack

**Nav2** is the primary navigation framework for ROS 2. It is a powerful and flexible suite of modular ROS 2 packages designed to help a robot autonomously navigate from a starting pose to a goal pose in an environment. Unlike simpler navigation systems, Nav2 is built with a behavior tree-based executive, allowing for complex decision-making and robust error handling.

![NAV2 Architecture](/img/NAV2.jpg)
![NAV2 Path Planning](/img/NAV2.png)

Key components of the Nav2 stack include:
-   **Localization**: `amcl` (Adaptive Monte Carlo Localization) or `robot_localization` (EKF/UKF) to determine the robot's pose.
-   **Costmaps**: `global_costmap` and `local_costmap` to represent the environment, including obstacles.
-   **Planners**: `global_planner` (e.g., A*, Dijkstra) to generate a path from start to goal, and `local_planner` (e.g., DWA, TEB) for local collision avoidance and trajectory execution.
-   **Controllers**: Execute the planned path while avoiding dynamic obstacles.
-   **Recovery Behaviors**: Strategies to recover from navigation failures (e.g., clear costmap, spin in place).
-   **Behavior Tree**: The executive that orchestrates all these components.

### 10.2 Configuring Nav2 for a Humanoid

While Nav2 is general-purpose, configuring it for a humanoid robot presents unique challenges, primarily due to bipedal locomotion. Traditional mobile robot planners assume holonomic or differential drive motion. Humanoids have complex gaits, balance constraints, and high centers of gravity.

Key considerations for humanoids:
-   **Kinematics and Dynamics**: Nav2 needs to understand the robot's non-holonomic motion constraints and balance capabilities. This might require custom plugins for local planners or specialized controllers.
-   **Footstep Planning**: For bipedal robots, navigation often involves discrete footstep planning rather than continuous path planning.
-   **Costmap Adaptation**: The costmaps might need to consider the robot's body shape, arm swings, and areas it can step over vs. areas it must avoid.

### 10.3 Path Planning

The Nav2 stack generates paths by considering the environment (from costmaps) and the robot's capabilities (from its kinematics and controller configuration).

#### Global Path Planning
The `global_planner` module generates an optimal path from the robot's current location to its goal. It uses algorithms like A* or Dijkstra to find the shortest or safest path through the static parts of the environment.

#### Local Path Planning
The `local_planner` continuously updates the path based on real-time sensor data, ensuring the robot avoids dynamic obstacles and executes the trajectory smoothly. Common local planners include:
-   **DWA (Dynamic Window Approach)**: Generates a "dynamic window" of possible velocities and chooses the best one.
-   **TEB (Timed Elastic Band)**: Optimizes the robot's trajectory by considering time, path length, and obstacle avoidance.

### 10.4 Obstacle Avoidance

Nav2's costmaps are central to obstacle avoidance.
-   **Costmap Layers**: The costmaps are composed of multiple layers (e.g., static map, obstacle layer, inflation layer).
-   **Sensor Integration**: Sensor data (LiDAR, depth cameras) is fed into the obstacle layer to mark occupied and free spaces.
-   **Inflation Layer**: Expands obstacles by the robot's radius to ensure safe clearance.

The combination of global and local planning, integrated with real-time sensor data and robust costmaps, allows a humanoid robot to navigate autonomously and safely through complex environments.
