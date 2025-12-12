---
title: Unity Physics System
sidebar_position: 4
---

# Section 3: Unity Physics System

## Learning Objectives
- Utilize Unity's PhysX engine for physics simulation
- Configure Rigidbody components and collision detection
- Implement custom physics materials and contact callbacks
- Optimize physics performance for real-time applications

## Content Outline

### Unity's Physics Architecture and PhysX Integration
Unity's physics system is built on NVIDIA's PhysX engine, providing robust and efficient physics simulation capabilities. The architecture includes:

#### Core Components
- **Physics Engine**: NVIDIA PhysX for collision detection and response
- **Collision Detection System**: Broad-phase and narrow-phase collision detection
- **Solver**: Constraint solving for joints and contacts
- **Broad-phase Acceleration**: Spatial partitioning for efficient collision detection

#### Physics Settings
Unity provides global physics settings accessible through Project Settings > Physics, including:
- Gravity settings
- Default material properties
- Solver iteration counts
- Layer collision matrix
- Contact offset and skin width

### Rigidbody Component Properties and Constraints
The Rigidbody component is essential for physics simulation in Unity:

#### Key Properties
- **Mass**: Mass of the object in kilograms
- **Drag**: Resistance to linear motion through the air
- **Angular Drag**: Resistance to rotational motion
- **Use Gravity**: Whether gravity affects this object
- **Is Kinematic**: Whether physics affects the object's motion
- **Interpolate**: How to smooth movement between frames
- **Collision Detection**: Collision detection mode for fast-moving objects

#### Constraints
Rigidbody constraints can lock specific degrees of freedom:
- Freeze Position (X, Y, Z)
- Freeze Rotation (X, Y, Z)

### Collider Types and Material Configurations
Unity supports various collider types for different geometric shapes:

#### Primitive Colliders
- **Box Collider**: Rectangular prism shape
- **Sphere Collider**: Spherical shape
- **Capsule Collider**: Capsule shape (common for characters)
- **Cylinder Collider**: Cylindrical shape
- **Plane Collider**: Infinite plane (one-sided)

#### Complex Colliders
- **Mesh Collider**: Uses the mesh geometry for collision
- **Terrain Collider**: Specialized for terrain objects
- **Wheel Collider**: Specialized for vehicle wheels

#### Physics Materials
Physics materials define surface properties:
- **Dynamic Friction**: Resistance when sliding
- **Static Friction**: Resistance to initial movement
- **Bounciness**: How bouncy the surface is
- **Friction Combine**: How friction combines with other surfaces
- **Bounce Combine**: How bounciness combines with other surfaces

### Physics Optimization Techniques for Performance
Efficient physics simulation requires careful optimization:

#### Performance Considerations
- Use appropriate collider types (primitive vs mesh)
- Adjust solver iteration counts based on requirements
- Optimize collision layers to reduce unnecessary checks
- Use object pooling for frequently created/destroyed physics objects
- Consider fixed timestep values for stability vs performance

## Diagram/Code Notes
```csharp
// Placeholder for Unity physics component code
// Example of custom physics material implementation
// Performance optimization techniques visualization
```

## Implementation Example: Unity Physics Components

```csharp
using UnityEngine;

public class PhysicsObject : MonoBehaviour
{
    [Header("Physics Properties")]
    public float mass = 1.0f;
    public float drag = 0.0f;
    public float angularDrag = 0.05f;
    public bool useGravity = true;
    public bool isKinematic = false;

    [Header("Collision Settings")]
    public CollisionDetectionMode collisionDetection = CollisionDetectionMode.Discrete;
    public RigidbodyInterpolation interpolation = RigidbodyInterpolation.None;

    private Rigidbody rb;

    void Start()
    {
        // Get or add Rigidbody component
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
        }

        // Configure Rigidbody properties
        ConfigureRigidbody();
    }

    void ConfigureRigidbody()
    {
        if (rb != null)
        {
            rb.mass = mass;
            rb.drag = drag;
            rb.angularDrag = angularDrag;
            rb.useGravity = useGravity;
            rb.isKinematic = isKinematic;
            rb.collisionDetectionMode = collisionDetection;
            rb.interpolation = interpolation;
        }
    }

    // Example of applying forces
    public void ApplyForce(Vector3 force, ForceMode mode = ForceMode.Force)
    {
        if (rb != null)
        {
            rb.AddForce(force, mode);
        }
    }

    // Example of applying torque
    public void ApplyTorque(Vector3 torque, ForceMode mode = ForceMode.Force)
    {
        if (rb != null)
        {
            rb.AddTorque(torque, mode);
        }
    }

    // Get current velocity
    public Vector3 GetVelocity()
    {
        return rb != null ? rb.velocity : Vector3.zero;
    }

    // Get current angular velocity
    public Vector3 GetAngularVelocity()
    {
        return rb != null ? rb.angularVelocity : Vector3.zero;
    }
}

// Physics Material Manager
public class PhysicsMaterialManager : MonoBehaviour
{
    [Header("Physics Materials")]
    public PhysicMaterial defaultMaterial;
    public PhysicMaterial highFrictionMaterial;
    public PhysicMaterial bouncyMaterial;

    [Header("Material Properties")]
    public float defaultDynamicFriction = 0.5f;
    public float defaultStaticFriction = 0.5f;
    public float defaultBounciness = 0.0f;

    void Start()
    {
        CreateDefaultMaterials();
    }

    void CreateDefaultMaterials()
    {
        if (defaultMaterial == null)
        {
            defaultMaterial = new PhysicMaterial("Default");
            defaultMaterial.dynamicFriction = defaultDynamicFriction;
            defaultMaterial.staticFriction = defaultStaticFriction;
            defaultMaterial.bounciness = defaultBounciness;
        }

        if (highFrictionMaterial == null)
        {
            highFrictionMaterial = new PhysicMaterial("HighFriction");
            highFrictionMaterial.dynamicFriction = 0.9f;
            highFrictionMaterial.staticFriction = 0.9f;
            highFrictionMaterial.bounciness = 0.0f;
        }

        if (bouncyMaterial == null)
        {
            bouncyMaterial = new PhysicMaterial("Bouncy");
            bouncyMaterial.dynamicFriction = 0.1f;
            bouncyMaterial.staticFriction = 0.1f;
            bouncyMaterial.bounciness = 0.8f;
        }
    }

    public void ApplyMaterial(GameObject obj, PhysicMaterial material)
    {
        Collider[] colliders = obj.GetComponents<Collider>();
        foreach (Collider col in colliders)
        {
            col.material = material;
        }
    }
}
```

## C# Example: Collision Detection and Response

```csharp
using UnityEngine;

public class CollisionHandler : MonoBehaviour
{
    [Header("Collision Events")]
    public bool enableCollisionEvents = true;
    public bool enableTriggerEvents = true;

    [Header("Audio/Visual Effects")]
    public AudioClip collisionSound;
    public GameObject collisionEffect;

    private AudioSource audioSource;

    void Start()
    {
        audioSource = GetComponent<AudioSource>();
        if (audioSource == null)
        {
            audioSource = gameObject.AddComponent<AudioSource>();
        }
    }

    // Called when this collider/rigidbody has begun touching another rigidbody/collider
    void OnCollisionEnter(Collision collision)
    {
        if (!enableCollisionEvents) return;

        // Get collision details
        Vector3 contactPoint = collision.contacts[0].point;
        Vector3 contactNormal = collision.contacts[0].normal;
        float impactForce = collision.relativeVelocity.magnitude;

        Debug.Log($"Collision with {collision.gameObject.name} at force: {impactForce}");

        // Play sound based on impact force
        if (audioSource != null && collisionSound != null)
        {
            audioSource.volume = Mathf.Clamp(impactForce / 10.0f, 0.1f, 1.0f);
            audioSource.PlayOneShot(collisionSound);
        }

        // Spawn visual effect
        if (collisionEffect != null)
        {
            GameObject effect = Instantiate(collisionEffect, contactPoint, Quaternion.LookRotation(contactNormal));
            Destroy(effect, 2.0f);
        }
    }

    // Called while this collider has been touching another rigidbody/collider
    void OnCollisionStay(Collision collision)
    {
        if (!enableCollisionEvents) return;

        // Handle continuous contact (e.g., friction effects)
    }

    // Called when this collider/rigidbody has stopped touching another rigidbody/collider
    void OnCollisionExit(Collision collision)
    {
        if (!enableCollisionEvents) return;

        Debug.Log($"Collision ended with {collision.gameObject.name}");
    }

    // Called when this collider enters a trigger collider
    void OnTriggerEnter(Collider other)
    {
        if (!enableTriggerEvents) return;

        Debug.Log($"Trigger entered: {other.gameObject.name}");
    }

    // Called while this collider stays inside a trigger collider
    void OnTriggerStay(Collider other)
    {
        if (!enableTriggerEvents) return;

        // Handle trigger stay (e.g., proximity detection)
    }

    // Called when this collider exits a trigger collider
    void OnTriggerExit(Collider other)
    {
        if (!enableTriggerEvents) return;

        Debug.Log($"Trigger exited: {other.gameObject.name}");
    }
}
```

## Key Concepts Summary
- Unity uses NVIDIA PhysX engine for physics simulation
- Rigidbody component controls physical behavior of objects
- Multiple collider types support different geometric shapes
- Physics materials define surface interaction properties
- Proper optimization is crucial for real-time performance

## References
- Unity Physics Manual
- NVIDIA PhysX Documentation
- Unity Scripting API Reference