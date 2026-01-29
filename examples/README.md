# BVH Integration Examples

This directory contains examples demonstrating how to use the BVH broad-phase collision detection system.

## bvh_integration_example.cpp

A complete example showing how to:
- Create a World and BroadPhaseManager
- Register rigid bodies with both the World and BroadPhase
- Run a physics simulation with automatic BVH-based collision culling
- Monitor collision detection performance

### Building

```bash
# From the root directory
g++ -O2 -Iinclude -o bvh_integration_example \
    ./src/body.cpp ./src/broad_phase.cpp ./src/collide_coarse.cpp \
    ./src/collide_fine.cpp ./src/contacts.cpp ./src/core.cpp \
    ./src/fgen.cpp ./src/joints.cpp ./src/particle.cpp \
    ./src/pcontacts.cpp ./src/pfgen.cpp ./src/plinks.cpp \
    ./src/pworld.cpp ./src/random.cpp ./src/world.cpp \
    ./examples/bvh_integration_example.cpp
```

### Running

```bash
./bvh_integration_example
```

## Key Concepts

1. **BroadPhaseManager Creation**: Create an instance with a capacity for potential contacts
2. **Body Registration**: Register bodies with both World (for simulation) and BroadPhase (for collision detection)
3. **Contact Generator Registration**: Register the BroadPhaseManager as a ContactGenerator with the World
4. **BVH Update**: Call `updateBVH()` before each call to `runPhysics()` to rebuild the hierarchy
5. **Automatic Integration**: The World automatically calls the BroadPhase during contact generation

## Performance Tips

- Choose an appropriate potential contact capacity based on scene density
- Update the BVH only when needed (e.g., after integration but before contact generation)
- Use appropriate bounding sphere radii - too large wastes performance, too small may miss contacts
- For mostly static scenes, consider implementing incremental BVH updates instead of full rebuilds
