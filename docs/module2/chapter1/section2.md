---
title: Gazebo Physics Engine Fundamentals
sidebar_position: 3
---

# Section 2: Gazebo Physics Engine Fundamentals

## Learning Objectives
- Configure Gazebo physics parameters (ODE, Bullet, SimBody)
- Set up gravity, damping, and friction coefficients
- Create and modify URDF/SDF models with physical properties
- Implement joint constraints and actuators

## Content Outline

### Gazebo Architecture and Physics Engine Options
Gazebo supports multiple physics engines, each with different characteristics:

#### ODE (Open Dynamics Engine)
- Default physics engine in many Gazebo versions
- Good performance for basic rigid body simulation
- Supports various joint types and collision detection
- Well-tested and stable for most applications

#### Bullet Physics
- More advanced physics simulation capabilities
- Better performance for complex collision scenarios
- Supports soft body dynamics and more complex constraints
- Good for applications requiring high accuracy

#### SimBody
- Stanford University's SimTK physics engine
- High-performance multibody dynamics
- Suitable for complex articulated systems
- Less commonly used but powerful for specific applications

### Configuration of Physics Parameters in World Files
Physics parameters are typically configured in SDF (Simulation Description Format) world files:

```xml
<sdf version="1.7">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

Key parameters include:
- `max_step_size`: Maximum time step for physics integration
- `real_time_factor`: Target simulation speed relative to real time
- `real_time_update_rate`: Updates per second
- `gravity`: Gravity vector (x, y, z)

### URDF/SDF Model Specifications with Inertial Properties
Models in Gazebo need proper inertial properties for realistic physics simulation:

#### Inertial Properties in URDF
```xml
<link name="link_name">
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0"
             iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
  <collision name="collision">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>
  <visual name="visual">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </visual>
</link>
```

### Joint Types and Constraint Implementation
Gazebo supports various joint types with different constraint properties:

#### Fixed Joint
- No degrees of freedom
- Rigidly connects two links

#### Revolute Joint
- One rotational degree of freedom
- Limited by upper/lower position limits

#### Prismatic Joint
- One translational degree of freedom
- Limited by position limits

#### Continuous Joint
- One rotational degree of freedom without limits

## Diagram/Code Notes
```xml
<!-- Placeholder for SDF world file example -->
<!-- Example URDF with physical properties -->
<!-- Code snippet for physics configuration -->
```

## Implementation Example: Physics Configuration

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="simple_box">
    <link name="box_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.0833333</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.0833333</iyy>
          <iyz>0.0</iyz>
          <izz>0.0833333</izz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
          <bounce>
            <restitution_coefficient>0.2</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1000000000000.0</kp>
              <kd>1.0</kd>
              <max_vel>100.0</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="visual">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
          <specular>0.8 0.2 0.2 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

## Python Example: Controlling Physics Parameters

```python
#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SetPhysicsProperties, GetPhysicsProperties
from gazebo_msgs.msg import ODEPhysics
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

def configure_physics():
    rospy.init_node('physics_configurator')

    # Wait for Gazebo services
    rospy.wait_for_service('/gazebo/set_physics_properties')

    try:
        # Get current physics properties
        get_physics = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        current_props = get_physics()

        # Set new physics properties
        set_physics = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)

        # Define new physics properties
        time_step = 0.001
        max_update_rate = 1000.0
        gravity = Vector3(0, 0, -9.8)
        ode_config = ODEPhysics()
        ode_config.auto_disable_bodies = False
        ode_config.sor_pgs_precon_iters = 0
        ode_config.sor_pgs_iters = 50
        ode_config.sor_pgs_w = 1.3
        ode_config.ode_error_reduction_parameter = 0.2
        ode_config.ode_error_reduction_velocity = 0.1
        ode_config.ode_constraint_force_mixing = 0.000001
        ode_config.ode_max_contacts = 20

        # Apply new physics properties
        resp = set_physics(
            time_step=time_step,
            max_update_rate=max_update_rate,
            gravity=gravity,
            ode_config=ode_config
        )

        if resp.success:
            print("Physics properties updated successfully")
        else:
            print(f"Failed to update physics properties: {resp.status_message}")

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == '__main__':
    configure_physics()
```

## Key Concepts Summary
- Gazebo supports multiple physics engines (ODE, Bullet, SimBody) with different characteristics
- Physics parameters are configured in SDF world files or through ROS services
- Proper inertial properties in URDF/SDF models are essential for realistic simulation
- Joint types and constraints determine how bodies can move relative to each other

## References
- Gazebo Documentation on Physics Engines
- ROS/Gazebo Integration Tutorials
- SDF Specification