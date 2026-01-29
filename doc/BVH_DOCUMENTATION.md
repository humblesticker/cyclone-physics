# BVH Broad-Phase Collision Detection

## Overview

The Bounding Volume Hierarchy (BVH) broad-phase collision detection system provides an efficient way to reduce the number of collision checks in physics simulations with many objects. Instead of checking every pair of objects (O(n²) complexity), the BVH organizes objects in a tree structure that allows for quick culling of pairs that cannot possibly collide.

## Features

- **Efficient Collision Culling**: Reduces the number of collision checks by organizing objects in a hierarchical tree structure
- **Dynamic Updates**: Supports adding, removing, and updating objects in the hierarchy
- **Seamless Integration**: Works as a ContactGenerator that can be integrated into the existing World pipeline
- **Performance Monitoring**: Provides methods to query the number of potential collision pairs

## Architecture

The BVH implementation consists of three main components:

### 1. BVHNode Template (include/cyclone/collide_coarse.h)

The existing `BVHNode<BoundingVolumeClass>` template provides the tree structure for organizing bounding volumes. It has been updated to work correctly with non-pointer bounding volume types like `BoundingSphere`.

Key methods:
- `insert()`: Adds a body to the hierarchy
- `getPotentialContacts()`: Traverses the tree to find potential collision pairs
- `recalculateBoundingVolume()`: Updates bounding volumes after changes

### 2. BroadPhaseManager (include/cyclone/broad_phase.h)

The `BroadPhaseManager` class manages the BVH and provides a high-level interface for broad-phase collision detection.

Key features:
- Maintains a BVH of all rigid bodies in the simulation
- Implements the `ContactGenerator` interface for integration with World
- Supports dynamic updates as objects move
- Can work with fine-grained collision detection generators

Key methods:
- `addBody(RigidBody* body, real radius)`: Adds a body with a bounding sphere
- `removeBody(RigidBody* body)`: Removes a body from the hierarchy
- `updateBVH()`: Rebuilds the hierarchy after objects have moved
- `addFineGenerator(ContactGenerator* gen)`: Registers fine-grained collision detectors
- `addContact(Contact* contact, unsigned limit)`: Generates contacts (ContactGenerator interface)
- `getPotentialContactCount()`: Returns the number of potential collision pairs

### 3. World Integration (include/cyclone/world.h)

The `World` class has been extended with methods to register bodies and contact generators:

New methods:
- `registerBody(RigidBody* body)`: Registers a body with the world
- `unregisterBody(RigidBody* body)`: Unregisters a body
- `registerContactGenerator(ContactGenerator* gen)`: Registers a contact generator
- `unregisterContactGenerator(ContactGenerator* gen)`: Unregisters a contact generator

## Usage Example

### Basic Setup

```cpp
#include <cyclone/cyclone.h>
#include <cyclone/broad_phase.h>

using namespace cyclone;

// Create a world
World world(1000);  // Max 1000 contacts

// Create a broad-phase manager
BroadPhaseManager* broadPhase = new BroadPhaseManager(1024);

// Create and add bodies
RigidBody body1, body2, body3;

// Position and configure bodies
body1.setPosition(0, 0, 0);
body1.setMass(1.0f);
body1.calculateDerivedData();

body2.setPosition(2, 0, 0);
body2.setMass(1.0f);
body2.calculateDerivedData();

body3.setPosition(10, 0, 0);
body3.setMass(1.0f);
body3.calculateDerivedData();

// Add bodies to BVH with bounding sphere radius
real radius = 1.0f;
broadPhase->addBody(&body1, radius);
broadPhase->addBody(&body2, radius);
broadPhase->addBody(&body3, radius);

// Register bodies with the world
world.registerBody(&body1);
world.registerBody(&body2);
world.registerBody(&body3);

// Register the broad-phase manager as a contact generator
world.registerContactGenerator(broadPhase);

// In your main loop:
while (running) {
    // Update the BVH before contact generation
    broadPhase->updateBVH();
    
    // Run physics (which will call generateContacts)
    world.runPhysics(duration);
}

// Cleanup
delete broadPhase;
```

### Integration with Fine-Grained Detection

```cpp
// Create fine-grained collision detectors
MyFineCollisionDetector* fineDetector = new MyFineCollisionDetector();

// Register the fine detector with the broad phase
broadPhase->addFineGenerator(fineDetector);

// Now the broad phase will pass potential pairs to the fine detector
```

## Performance Characteristics

### Time Complexity

- **Build**: O(n log n) - Initial construction of the tree
- **Update**: O(n log n) - Rebuilding the tree after movement
- **Query**: O(log n) per query - Finding potential contacts

### Space Complexity

- **Memory**: O(n) - Linear in the number of objects

### Performance Comparison

For n objects:
- **Without BVH**: n×(n-1)/2 collision checks (O(n²))
- **With BVH**: Typically O(n log n) to O(n) collision checks

Example: For 400 objects:
- Without BVH: 79,800 pairwise checks
- With BVH: Typically 5-50 potential pairs (99%+ reduction)

## Implementation Details

### Update Strategy

The current implementation uses a "rebuild-per-frame" strategy for dynamic scenes:

1. After bodies are integrated but before contact generation
2. The entire BVH is rebuilt from scratch
3. This is simpler and often faster than incremental updates for highly dynamic scenes

Alternative strategies can be implemented:
- Incremental updates for mostly static scenes
- Lazy rebuilding based on a quality metric
- Partial tree rotations to maintain balance

### Bounding Volume Selection

The implementation uses bounding spheres (`BoundingSphere`) because they:
- Are simple and fast to compute
- Are rotation-invariant (no need to update when objects rotate)
- Provide good culling efficiency for many scenarios

Other bounding volumes (AABB, OBB) can be used by instantiating the `BVHNode` template with different types.

### Tree Construction

The tree is built using a greedy insertion strategy:
- Start with the first object as the root
- For each subsequent object, insert it where it causes the least growth
- This produces reasonably balanced trees without expensive rebalancing

## Testing

A comprehensive test suite is provided in `src/demos/bvhtest/bvhtest.cpp`. To run the tests:

```bash
# Compile the test
make -f linuxmake.mk bvhtest

# Or compile standalone
g++ -O2 -Iinclude -o bvhtest_standalone \
    ./src/body.cpp ./src/broad_phase.cpp ./src/collide_coarse.cpp \
    ./src/collide_fine.cpp ./src/contacts.cpp ./src/core.cpp \
    ./src/fgen.cpp ./src/joints.cpp ./src/particle.cpp \
    ./src/pcontacts.cpp ./src/pfgen.cpp ./src/plinks.cpp \
    ./src/pworld.cpp ./src/random.cpp ./src/world.cpp \
    ./src/demos/bvhtest/bvhtest.cpp

# Run the test
./bvhtest_standalone
```

The test validates:
1. BVH creation and body insertion
2. Potential contact detection
3. BVH updates after body movement
4. Body removal
5. Performance with large numbers of objects

## Future Enhancements

Possible improvements for future versions:

1. **Incremental Updates**: Support for updating only the affected portions of the tree
2. **Multi-threading**: Parallel BVH construction and traversal
3. **Self-Collisions**: Better handling of objects that need to check for self-collision
4. **Continuous Collision Detection**: Integration with swept volume techniques
5. **Alternative Bounding Volumes**: Support for AABBs, OBBs, and capsules
6. **Adaptive Strategies**: Automatic selection of rebuild vs. update based on scene dynamics

## References

- Goldsmith, J. and Salmon, J. (1987). "Automatic Creation of Object Hierarchies for Ray Tracing"
- Ericson, C. (2004). "Real-Time Collision Detection", Chapter 6
- Cyclone Physics Engine Documentation

## Compatibility

The BVH broad-phase system is fully compatible with:
- Existing fine-grained collision detection routines
- The World simulation pipeline
- All existing contact generators
- Legacy code that doesn't use the BVH (opt-in feature)

No breaking changes were made to existing APIs.
