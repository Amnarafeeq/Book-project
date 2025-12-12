# Module 2: The Digital Twin (Gazebo & Unity)

## Introduction
Digital twins are virtual replicas of physical systems that enable testing, validation, and development of robotic applications without requiring physical hardware. This module explores two leading simulation platforms: Gazebo for physics-based simulation and Unity for advanced 3D environments and AI training.

## Skills You Will Learn
- Creating and configuring robot models in Gazebo
- Setting up realistic simulation environments
- Implementing physics-based sensors and actuators
- Building 3D environments in Unity for robotics
- Integrating Unity with ROS 2 using ROS# or similar bridges
- Training AI agents in simulation environments

## Tools You Will Use
- Gazebo Garden or Fortress
- Unity 3D (Personal or Pro)
- Robot Operating System 2 (ROS 2)
- Unity ROS# package
- NVIDIA Omniverse (optional advanced tool)
- Physics engines (ODE, Bullet, SDFormat)

## Example Code

```xml
<!-- Example URDF robot model for Gazebo -->
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Gazebo plugin for ROS control -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_diameter>0.1</wheel_diameter>
    </plugin>
  </gazebo>
</robot>
```

## Key Concepts

### Gazebo Simulation
Gazebo provides high-fidelity physics simulation with realistic sensor models, making it ideal for testing robotic algorithms before deployment on real hardware.

### Unity for Robotics
Unity offers advanced 3D rendering capabilities and is increasingly used for creating photorealistic environments for training computer vision and AI systems.

### Simulation-to-Reality Gap
Understanding the differences between simulation and reality is crucial for developing robust robotic systems that transfer effectively to the physical world.

## Diagrams
![Digital Twin Architecture](/img/digital-twin-architecture.png)

:::note
Simulation environments allow for rapid prototyping and testing of robotic algorithms without the constraints and risks associated with physical hardware.
:::

:::tip
When designing simulation environments, focus on modeling the aspects of reality that are most critical to your specific application.
:::

## Next Steps
After completing this module, you will understand how to create and use digital twins for robotic development, preparing you for advanced AI integration in Module 3.