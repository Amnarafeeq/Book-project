---
title: Gravity and Environmental Forces
sidebar_position: 5
---

# Section 4: Gravity and Environmental Forces

## Learning Objectives
- Implement custom gravity fields and environmental forces
- Simulate atmospheric effects and fluid dynamics
- Model friction and damping in different environments
- Handle multi-body gravitational systems

## Content Outline

### Configuring Standard and Custom Gravity Parameters
Gravity is a fundamental force in physics simulation that affects all objects with mass. Both Gazebo and Unity provide mechanisms to configure gravity parameters:

#### Standard Gravity Configuration
In most simulations, gravity is configured as a constant vector pointing downward:
- Earth's gravity: approximately 9.81 m/s² in the negative Y direction
- Other celestial bodies: varies (Mars: 3.71 m/s², Moon: 1.62 m/s²)

#### Custom Gravity Fields
For more complex scenarios, custom gravity fields can be implemented:
- Radial gravity for spherical bodies
- Variable gravity based on position
- Time-varying gravitational fields

### Implementing Environmental Forces (Wind, Drag, Buoyancy)
Environmental forces add realism to physics simulations:

#### Wind Forces
Wind can be simulated as a directional force field:
- Constant wind: uniform force in a specific direction
- Turbulent wind: varying force with time and position
- Wind zones: localized areas with specific wind characteristics

#### Drag Forces
Drag represents resistance when objects move through fluids:
- Linear drag: proportional to velocity
- Quadratic drag: proportional to velocity squared
- Shape-dependent drag coefficients

#### Buoyancy Forces
Buoyancy simulates the upward force on objects in fluids:
- Based on displaced fluid volume
- Affected by fluid density
- Can cause floating or sinking behavior

### Friction and Damping Models
Friction and damping models affect how objects interact with surfaces and environments:

#### Friction Models
- Static friction: resistance to initial motion
- Dynamic friction: resistance during motion
- Anisotropic friction: different friction in different directions

#### Damping Models
- Linear damping: velocity-proportional resistance
- Angular damping: rotation-proportional resistance
- Material-dependent damping coefficients

### Multi-Body Gravitational Simulation
For complex systems with multiple gravitationally-interacting bodies:

#### N-Body Simulation
- Newtonian gravity between all pairs of bodies
- Computational complexity: O(n²) for n bodies
- Approximation methods for large systems

#### Performance Considerations
- Hierarchical methods (e.g., Barnes-Hut algorithm)
- Softening parameters to prevent numerical instabilities
- Adaptive time stepping for systems with varying dynamics

## Diagram/Code Notes
```
// Placeholder for custom gravity field implementation
// Diagram showing force vector calculations
// Code example for environmental force simulation
```

## Implementation Example: Custom Gravity and Environmental Forces

```python
import numpy as np
from scipy.spatial.distance import pdist, squareform

class EnvironmentalForces:
    """Class to handle various environmental forces in simulation"""

    def __init__(self, gravity_vector=np.array([0, -9.81, 0])):
        self.gravity = gravity_vector
        self.wind_zones = []
        self.fluid_zones = []

    def add_wind_zone(self, position, direction, strength, radius):
        """Add a localized wind zone"""
        self.wind_zones.append({
            'position': np.array(position),
            'direction': np.array(direction) / np.linalg.norm(direction),  # Normalize
            'strength': strength,
            'radius': radius
        })

    def add_fluid_zone(self, position, size, density, viscosity):
        """Add a fluid zone with buoyancy and drag properties"""
        self.fluid_zones.append({
            'position': np.array(position),
            'size': np.array(size),
            'density': density,
            'viscosity': viscosity
        })

    def calculate_gravity(self, mass):
        """Calculate gravitational force on an object"""
        return mass * self.gravity

    def calculate_wind_force(self, position, velocity, object_area=1.0):
        """Calculate wind force on an object"""
        total_wind_force = np.zeros(3)

        for zone in self.wind_zones:
            # Calculate distance to wind zone
            dist = np.linalg.norm(position - zone['position'])

            if dist <= zone['radius']:
                # Calculate force based on distance (inverse square or linear falloff)
                falloff = max(0, 1 - dist / zone['radius'])
                wind_effect = zone['direction'] * zone['strength'] * falloff
                total_wind_force += wind_effect * object_area

        return total_wind_force

    def calculate_drag_force(self, velocity, object_area=1.0, drag_coefficient=1.0, fluid_density=1.225):
        """Calculate drag force (air resistance)"""
        speed = np.linalg.norm(velocity)
        if speed == 0:
            return np.zeros(3)

        drag_magnitude = 0.5 * fluid_density * speed**2 * drag_coefficient * object_area
        drag_direction = -velocity / speed  # Opposite to velocity direction
        return drag_direction * drag_magnitude

    def calculate_buoyancy_force(self, position, volume, object_density):
        """Calculate buoyancy force for objects in fluid"""
        for zone in self.fluid_zones:
            # Check if object is within fluid zone
            pos_min = zone['position'] - zone['size'] / 2
            pos_max = zone['position'] + zone['size'] / 2

            if (pos_min[0] <= position[0] <= pos_max[0] and
                pos_min[1] <= position[1] <= pos_max[1] and
                pos_min[2] <= position[2] <= pos_max[2]):

                # Buoyancy = weight of displaced fluid
                displaced_fluid_weight = zone['density'] * volume * np.linalg.norm(self.gravity)
                buoyancy_direction = -self.gravity / np.linalg.norm(self.gravity)  # Opposite to gravity
                return buoyancy_direction * displaced_fluid_weight

        return np.zeros(3)  # No buoyancy if not in fluid

    def calculate_all_forces(self, position, velocity, mass, volume, object_area=1.0,
                           drag_coefficient=1.0, object_density=1000):
        """Calculate all environmental forces on an object"""
        total_force = np.zeros(3)

        # Add gravity
        total_force += self.calculate_gravity(mass)

        # Add wind force
        total_force += self.calculate_wind_force(position, velocity, object_area)

        # Add drag force
        total_force += self.calculate_drag_force(velocity, object_area, drag_coefficient)

        # Add buoyancy force
        total_force += self.calculate_buoyancy_force(position, volume, object_density)

        return total_force

# Example usage
env_forces = EnvironmentalForces(gravity_vector=np.array([0, -3.71, 0]))  # Mars gravity

# Add a wind zone
env_forces.add_wind_zone(position=[5, 0, 0], direction=[1, 0, 0], strength=2.0, radius=3.0)

# Add a fluid zone (water)
env_forces.add_fluid_zone(position=[0, -2, 0], size=[10, 4, 10], density=1000, viscosity=0.001)

# Calculate forces on an object
position = np.array([6, 0, 0])
velocity = np.array([1, 0, 0])
mass = 1.0
volume = 0.001  # 1 liter

total_force = env_forces.calculate_all_forces(position, velocity, mass, volume)
print(f"Total environmental forces: {total_force}")
```

## Gazebo Implementation: Custom Gravity Plugin

```cpp
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class CustomGravityPlugin : public WorldPlugin
  {
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      this->world = _world;

      // Check if custom gravity parameters are provided
      if (_sdf->HasElement("gravity_x"))
        this->gravity.X() = _sdf->Get<double>("gravity_x");
      if (_sdf->HasElement("gravity_y"))
        this->gravity.Y() = _sdf->Get<double>("gravity_y");
      if (_sdf->HasElement("gravity_z"))
        this->gravity.Z() = _sdf->Get<double>("gravity_z");

      // Set the custom gravity
      this->world->SetGravity(this->gravity);

      // Connect to pre-update event
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CustomGravityPlugin::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      // Custom gravity update logic could go here
      // For example, time-varying gravity or position-dependent gravity
    }

    private: physics::WorldPtr world;
    private: math::Vector3 gravity = math::Vector3(0, 0, -9.8);
    private: event::ConnectionPtr updateConnection;
  };

  GZ_REGISTER_WORLD_PLUGIN(CustomGravityPlugin)
}
```

## Unity Implementation: Environmental Forces Manager

```csharp
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class EnvironmentalForces : MonoBehaviour
{
    [Header("Gravity Settings")]
    public Vector3 customGravity = new Vector3(0, -9.81f, 0);
    public bool useCustomGravity = true;

    [Header("Wind Settings")]
    public float windForce = 0f;
    public Vector3 windDirection = Vector3.right;
    public bool enableWind = false;

    [Header("Fluid Settings")]
    public float fluidDensity = 1.225f; // Air density
    public float dragCoefficient = 1f;
    public float objectArea = 1f;
    public bool isSubmerged = false;
    public float submergedRatio = 0f;

    private Rigidbody rb;
    private Vector3 dragForce = Vector3.zero;
    private Vector3 buoyancyForce = Vector3.zero;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
        }
    }

    void FixedUpdate()
    {
        if (useCustomGravity)
        {
            // Apply custom gravity
            rb.AddForce(customGravity * rb.mass, ForceMode.Force);
        }

        if (enableWind)
        {
            // Apply wind force
            Vector3 wind = windDirection.normalized * windForce;
            rb.AddForce(wind, ForceMode.Force);
        }

        // Calculate and apply drag force
        CalculateDragForce();
        rb.AddForce(dragForce, ForceMode.Force);

        // Apply buoyancy if submerged
        if (isSubmerged)
        {
            CalculateBuoyancyForce();
            rb.AddForce(buoyancyForce, ForceMode.Force);
        }
    }

    void CalculateDragForce()
    {
        // Drag force calculation: F = 0.5 * rho * v^2 * Cd * A
        Vector3 velocity = rb.velocity;
        float speedSqr = velocity.sqrMagnitude;

        if (speedSqr > 0)
        {
            float dragMagnitude = 0.5f * fluidDensity * speedSqr * dragCoefficient * objectArea;
            dragForce = -velocity.normalized * dragMagnitude;
        }
        else
        {
            dragForce = Vector3.zero;
        }
    }

    void CalculateBuoyancyForce()
    {
        // Buoyancy force: F = rho * V * g
        // Simplified calculation - in real applications,
        // this would consider the volume of fluid displaced
        float submergedVolume = rb.mass / rb.drag * submergedRatio; // Simplified
        float buoyancyMagnitude = fluidDensity * submergedVolume * Mathf.Abs(customGravity.y);

        buoyancyForce = -customGravity.normalized * buoyancyMagnitude;
    }

    // Method to set submersion level
    public void SetSubmersion(float ratio)
    {
        submergedRatio = Mathf.Clamp01(ratio);
        isSubmerged = submergedRatio > 0f;
    }

    // Method to apply additional environmental forces
    public void ApplyEnvironmentalForce(Vector3 force)
    {
        rb.AddForce(force, ForceMode.Force);
    }

    // Visualize forces for debugging
    void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying) return;

        Gizmos.color = Color.red;
        Gizmos.DrawRay(transform.position, customGravity * 10f); // Gravity vector

        if (enableWind)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(transform.position, windDirection * windForce * 10f); // Wind vector
        }

        Gizmos.color = Color.yellow;
        Gizmos.DrawRay(transform.position, dragForce); // Drag vector

        if (isSubmerged)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawRay(transform.position, buoyancyForce); // Buoyancy vector
        }
    }
}
```

## Key Concepts Summary
- Gravity can be customized in both Gazebo and Unity for different environments
- Environmental forces (wind, drag, buoyancy) add realism to simulations
- Proper modeling of friction and damping is crucial for realistic behavior
- Multi-body gravitational systems require special computational approaches

## References
- Gazebo Custom Plugins Documentation
- Unity Physics Documentation
- "Real-Time Rendering" by Tomas Akenine-Möller