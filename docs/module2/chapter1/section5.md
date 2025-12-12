---
title: Collision Detection and Response
sidebar_position: 6
---

# Section 5: Collision Detection and Response

## Learning Objectives
- Implement broad-phase and narrow-phase collision detection
- Configure collision response parameters (restitution, friction)
- Handle complex collision shapes and compound objects
- Optimize collision performance in dense environments

## Content Outline

### Collision Detection Pipeline
The collision detection process in physics engines typically follows a two-phase approach:

#### Broad-Phase Collision Detection
- Purpose: Quickly eliminate pairs of objects that are definitely not colliding
- Methods:
  - Spatial partitioning (octrees, quadtrees, spatial grids)
  - Sweep and prune algorithms
  - Bounding volume hierarchies (BVH)
- Performance: O(n) or O(n log n) complexity depending on algorithm

#### Narrow-Phase Collision Detection
- Purpose: Precisely determine if two potentially colliding objects actually intersect
- Methods:
  - Separating Axis Theorem (SAT) for convex shapes
  - GJK algorithm for convex shapes
  - Triangle-triangle intersection for meshes
- Output: Contact points, penetration depth, collision normals

#### Contact Resolution
- Purpose: Calculate appropriate response forces to prevent objects from penetrating
- Methods:
  - Impulse-based solvers
  - Position-based constraints
  - Penalty methods

### Shape Representation
Different geometric representations serve different purposes in collision detection:

#### Primitive Shapes
- **Sphere**: Fastest collision detection, good for rough approximations
- **Box**: Good for rectangular objects, reasonably fast
- **Capsule**: Good for characters, reasonably fast
- **Cylinder**: Useful for specific applications

#### Complex Shapes
- **Convex Hull**: Fast collision detection for complex convex shapes
- **Triangle Mesh**: Most accurate but computationally expensive
- **Compound Shapes**: Multiple primitive shapes combined for complex objects

### Contact Manifold Generation and Response
When collisions occur, the physics engine generates contact manifolds:

#### Contact Points
- Points where objects touch or penetrate
- Used to apply forces to prevent further penetration
- Number of points affects stability of contact

#### Contact Normals
- Surface normals at contact points
- Determine direction of response forces
- Critical for proper sliding and rolling behavior

#### Penetration Depth
- Measure of how much objects overlap
- Used to calculate correction forces
- Should be kept minimal for stable simulation

### Performance Considerations and Optimization
Efficient collision detection is crucial for real-time applications:

#### Optimization Strategies
- Use simpler collision shapes where possible
- Implement spatial partitioning for large scenes
- Use multi-resolution collision detection
- Adjust solver parameters for performance vs accuracy

#### Common Performance Issues
- Excessive contact points
- Deep penetrations requiring large corrections
- Unstable joints or constraints

## Diagram/Code Notes
```cpp
// Placeholder for collision detection algorithm
// Example of compound collision shape setup
// Performance benchmarking code
```

## Implementation Example: Collision Detection System

```cpp
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>

// Basic 3D vector class
struct Vector3 {
    float x, y, z;

    Vector3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }

    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }

    float dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    float length() const {
        return std::sqrt(x*x + y*y + z*z);
    }

    Vector3 normalize() const {
        float len = length();
        if (len > 0) {
            return Vector3(x/len, y/len, z/len);
        }
        return Vector3(0, 0, 0);
    }
};

// Base collision shape class
class CollisionShape {
public:
    virtual ~CollisionShape() = default;
    virtual bool intersects(const CollisionShape& other, Vector3& contactPoint, Vector3& normal) const = 0;
    virtual float getVolume() const = 0;
};

// Sphere collision shape
class SphereShape : public CollisionShape {
public:
    Vector3 center;
    float radius;

    SphereShape(const Vector3& center, float radius) : center(center), radius(radius) {}

    bool intersects(const CollisionShape& other, Vector3& contactPoint, Vector3& normal) const override {
        // For simplicity, only check sphere-sphere collision
        const SphereShape* otherSphere = dynamic_cast<const SphereShape*>(&other);
        if (otherSphere) {
            Vector3 diff = otherSphere->center - this->center;
            float distance = diff.length();
            float minDistance = this->radius + otherSphere->radius;

            if (distance < minDistance) {
                // Collision detected
                contactPoint = this->center + diff.normalize() * this->radius;
                normal = diff.normalize();
                return true;
            }
        }
        return false;
    }

    float getVolume() const override {
        return (4.0f/3.0f) * 3.14159f * radius * radius * radius;
    }
};

// Axis-Aligned Bounding Box (AABB)
class AABBShape : public CollisionShape {
public:
    Vector3 minPoint;
    Vector3 maxPoint;

    AABBShape(const Vector3& minPoint, const Vector3& maxPoint)
        : minPoint(minPoint), maxPoint(maxPoint) {}

    bool intersects(const CollisionShape& other, Vector3& contactPoint, Vector3& normal) const override {
        // For simplicity, only check AABB-AABB collision
        const AABBShape* otherAABB = dynamic_cast<const AABBShape*>(&other);
        if (otherAABB) {
            // Check for overlap in all three dimensions
            bool overlapX = (this->minPoint.x <= otherAABB->maxPoint.x) &&
                           (otherAABB->minPoint.x <= this->maxPoint.x);
            bool overlapY = (this->minPoint.y <= otherAABB->maxPoint.y) &&
                           (otherAABB->minPoint.y <= this->maxPoint.y);
            bool overlapZ = (this->minPoint.z <= otherAABB->maxPoint.z) &&
                           (otherAABB->minPoint.z <= this->maxPoint.z);

            if (overlapX && overlapY && overlapZ) {
                // Collision detected - compute contact point
                contactPoint.x = (this->maxPoint.x + otherAABB->minPoint.x) / 2.0f;
                contactPoint.y = (this->maxPoint.y + otherAABB->minPoint.y) / 2.0f;
                contactPoint.z = (this->maxPoint.z + otherAABB->minPoint.z) / 2.0f;

                // Simple normal calculation (this is simplified)
                normal = Vector3(1, 0, 0); // Placeholder
                return true;
            }
        }
        return false;
    }

    float getVolume() const override {
        Vector3 size = maxPoint - minPoint;
        return size.x * size.y * size.z;
    }
};

// Collision manager class
class CollisionManager {
private:
    std::vector<std::unique_ptr<CollisionShape>> shapes;

public:
    void addShape(std::unique_ptr<CollisionShape> shape) {
        shapes.push_back(std::move(shape));
    }

    struct CollisionPair {
        int index1;
        int index2;
        Vector3 contactPoint;
        Vector3 normal;
    };

    std::vector<CollisionPair> detectCollisions() {
        std::vector<CollisionPair> collisions;

        // Brute force approach - O(n²) - for demonstration
        // In practice, spatial partitioning would be used
        for (size_t i = 0; i < shapes.size(); ++i) {
            for (size_t j = i + 1; j < shapes.size(); ++j) {
                Vector3 contactPoint, normal;
                if (shapes[i]->intersects(*shapes[j], contactPoint, normal)) {
                    collisions.push_back({static_cast<int>(i), static_cast<int>(j), contactPoint, normal});
                }
            }
        }

        return collisions;
    }

    size_t getShapeCount() const {
        return shapes.size();
    }
};

// Example usage
#include <iostream>

int main() {
    CollisionManager manager;

    // Add some shapes
    manager.addShape(std::make_unique<SphereShape>(Vector3(0, 0, 0), 1.0f));
    manager.addShape(std::make_unique<SphereShape>(Vector3(1.5f, 0, 0), 1.0f)); // Should collide
    manager.addShape(std::make_unique<AABBShape>(Vector3(-2, -1, -1), Vector3(-1, 1, 1)));

    // Detect collisions
    auto collisions = manager.detectCollisions();

    std::cout << "Found " << collisions.size() << " collisions:\n";
    for (const auto& collision : collisions) {
        std::cout << "  Shape " << collision.index1 << " collides with Shape " << collision.index2
                  << " at (" << collision.contactPoint.x << ", " << collision.contactPoint.y
                  << ", " << collision.contactPoint.z << ")\n";
    }

    return 0;
}
```

## Gazebo Implementation: Collision Detection Configuration

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="collision_example">
    <link name="link1">
      <collision name="collision1">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <surface>
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
          <bounce>
            <restitution_coefficient>0.5</restitution_coefficient>
            <threshold>100000</threshold>
          </bounce>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
      <visual name="visual1">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>
```

## Unity Implementation: Collision Detection and Response

```csharp
using UnityEngine;

public class CollisionResponse : MonoBehaviour
{
    [Header("Collision Response Settings")]
    public float restitution = 0.5f; // Bounciness (0 = no bounce, 1 = perfect bounce)
    public float staticFriction = 0.5f; // Friction when object is at rest
    public float dynamicFriction = 0.3f; // Friction when object is moving
    public float contactTolerance = 0.01f; // How close objects can get before collision

    [Header("Performance Settings")]
    public int maxContactPoints = 4; // Maximum number of contact points to process
    public float collisionCooldown = 0.1f; // Time between collision events

    private Rigidbody rb;
    private float lastCollisionTime = 0f;
    private bool isColliding = false;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        if (Time.time - lastCollisionTime < collisionCooldown) return;

        lastCollisionTime = Time.time;

        // Process collision contacts
        foreach (ContactPoint contact in collision.contacts)
        {
            if (collision.contacts.Length <= maxContactPoints)
            {
                ProcessContact(contact, collision.relativeVelocity.magnitude);
            }
        }

        isColliding = true;
    }

    void OnCollisionStay(Collision collision)
    {
        // Handle continuous contact (e.g., sliding friction)
        foreach (ContactPoint contact in collision.contacts)
        {
            ApplyFrictionForce(contact, collision.relativeVelocity);
        }
    }

    void OnCollisionExit(Collision collision)
    {
        isColliding = false;
    }

    void ProcessContact(ContactPoint contact, float impactVelocity)
    {
        // Calculate normal force based on restitution
        Vector3 relativeVelocity = rb.GetPointVelocity(contact.point);
        float normalVelocity = Vector3.Dot(relativeVelocity, contact.normal);

        if (normalVelocity < 0) // Object moving into contact
        {
            // Calculate impulse for bounce
            float impulse = -(1 + restitution) * normalVelocity;

            // Apply bounce impulse
            Vector3 bounceImpulse = contact.normal * impulse;
            rb.AddForceAtPosition(bounceImpulse, contact.point, ForceMode.Impulse);

            // Debug visualization
            Debug.DrawRay(contact.point, contact.normal * 0.5f, Color.red, 2.0f);
        }
    }

    void ApplyFrictionForce(ContactPoint contact, Vector3 relativeVelocity)
    {
        // Calculate friction force perpendicular to normal
        Vector3 normalVelocity = Vector3.Project(relativeVelocity, contact.normal);
        Vector3 tangentialVelocity = relativeVelocity - normalVelocity;

        float tangentialSpeed = tangentialVelocity.magnitude;
        if (tangentialSpeed > 0.01f) // Only apply friction if moving
        {
            float frictionCoefficient = dynamicFriction; // Use dynamic friction when moving

            // Use static friction if object is nearly stopped
            if (tangentialSpeed < 0.1f)
            {
                frictionCoefficient = staticFriction;
            }

            Vector3 frictionDirection = -tangentialVelocity.normalized;
            Vector3 frictionForce = frictionDirection * tangentialSpeed * frictionCoefficient;

            rb.AddForceAtPosition(frictionForce, contact.point, ForceMode.Force);
        }
    }

    // Method to configure collision response at runtime
    public void SetCollisionProperties(float newRestitution, float newStaticFriction, float newDynamicFriction)
    {
        restitution = Mathf.Clamp01(newRestitution);
        staticFriction = Mathf.Clamp01(newStaticFriction);
        dynamicFriction = Mathf.Clamp01(newDynamicFriction);
    }

    // Visualization of collision properties
    void OnValidate()
    {
        restitution = Mathf.Clamp01(restitution);
        staticFriction = Mathf.Clamp01(staticFriction);
        dynamicFriction = Mathf.Clamp01(dynamicFriction);
    }

    // Get collision status
    public bool IsInCollision()
    {
        return isColliding;
    }

    // Calculate approximate contact area (simplified)
    public float GetContactArea(Collision collision)
    {
        if (collision.contacts.Length == 0) return 0f;

        // Simplified approach - in reality, contact area calculation is complex
        float totalArea = 0f;
        foreach (ContactPoint contact in collision.contacts)
        {
            // This is a simplified approximation
            totalArea += contact.separation * contact.separation; // Rough area approximation
        }

        return totalArea;
    }
}
```

## Key Concepts Summary
- Collision detection uses a two-phase approach: broad-phase and narrow-phase
- Different shape representations offer trade-offs between accuracy and performance
- Contact manifolds define how objects respond to collisions
- Proper configuration of collision parameters is essential for realistic behavior
- Performance optimization is critical for real-time applications

## References
- "Real-Time Collision Detection" by Christer Ericson
- Gazebo Collision Detection Documentation
- Unity Collision Detection Documentation
- "Game Physics Engine Development" by Ian Millington