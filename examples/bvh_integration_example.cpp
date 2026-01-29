/*
 * BVH Integration Example
 *
 * This example demonstrates how to integrate the BVH broad-phase
 * collision detection system with the World class for efficient
 * collision detection in a multi-body simulation.
 *
 * Part of the Cyclone physics system.
 */

#include <cyclone/cyclone.h>
#include <stdio.h>

using namespace cyclone;

int main(int argc, char** argv)
{
    printf("BVH Integration Example\n");
    printf("=======================\n\n");

    // Create a world with capacity for 1000 contacts
    World world(1000);

    // Create a broad-phase manager with capacity for 2048 potential contacts
    BroadPhaseManager broadPhase(2048);

    // Create some rigid bodies
    const int numBodies = 50;
    RigidBody* bodies = new RigidBody[numBodies];

    printf("Setting up %d rigid bodies...\n", numBodies);

    // Position bodies in a grid and configure them
    const real spacing = 2.0f;
    const real radius = 1.0f;
    int gridSize = 7;  // 7x7 = 49 bodies (close to 50)
    
    for (int i = 0; i < numBodies; i++)
    {
        int x = i % gridSize;
        int z = i / gridSize;
        
        // Set position
        bodies[i].setPosition(
            x * spacing - (gridSize * spacing) / 2.0f,
            10.0f,  // Start above ground
            z * spacing - (gridSize * spacing) / 2.0f
        );
        
        // Set mass and inertia
        bodies[i].setMass(1.0f);
        
        Matrix3 inertiaTensor;
        inertiaTensor.setDiagonal(0.4f, 0.4f, 0.4f);  // Approximate for sphere
        bodies[i].setInertiaTensor(inertiaTensor);
        
        // Set damping to stabilize simulation
        bodies[i].setDamping(0.95f, 0.8f);
        
        // Set acceleration (gravity)
        bodies[i].setAcceleration(0, -9.81f, 0);
        
        // Calculate derived data
        bodies[i].calculateDerivedData();
        bodies[i].setAwake(true);
        
        // Register with world
        world.registerBody(&bodies[i]);
        
        // Add to broad-phase
        broadPhase.addBody(&bodies[i], radius);
    }

    printf("  All bodies registered with World and BroadPhase\n");

    // Register the broad-phase manager as a contact generator with the world
    world.registerContactGenerator(&broadPhase);
    
    printf("  BroadPhase registered as contact generator\n");

    // Simulation parameters
    const real timeStep = 1.0f / 60.0f;  // 60 FPS
    const int numFrames = 100;
    
    printf("\nRunning simulation for %d frames...\n", numFrames);

    // Run simulation
    for (int frame = 0; frame < numFrames; frame++)
    {
        // Start frame (clear accumulators)
        world.startFrame();
        
        // Apply any forces here
        // (gravity is already set as acceleration on each body)
        
        // Update BVH before contact generation
        broadPhase.updateBVH();
        
        // Run physics (integrates bodies, generates contacts, resolves contacts)
        world.runPhysics(timeStep);
        
        // Every 20 frames, print status
        if (frame % 20 == 0)
        {
            unsigned potentialContacts = broadPhase.getPotentialContactCount();
            
            printf("  Frame %3d: %d potential collision pairs detected\n", 
                   frame, potentialContacts);
            
            // Print position of first body as example
            Vector3 pos = bodies[0].getPosition();
            printf("    Body[0] position: (%.2f, %.2f, %.2f)\n", 
                   pos.x, pos.y, pos.z);
        }
    }

    printf("\nSimulation complete!\n");
    
    // Print final statistics
    printf("\nFinal Statistics:\n");
    printf("  Bodies simulated: %d\n", numBodies);
    printf("  Frames simulated: %d\n", numFrames);
    
    unsigned maxPairwise = (numBodies * (numBodies - 1)) / 2;
    unsigned avgBroadPhase = broadPhase.getPotentialContactCount();
    
    printf("  Max possible collision pairs: %d\n", maxPairwise);
    printf("  Broad-phase collision pairs: %d\n", avgBroadPhase);
    printf("  Collision check reduction: %.1f%%\n", 
           100.0f * (1.0f - (float)avgBroadPhase / (float)maxPairwise));

    // Cleanup
    delete[] bodies;

    printf("\nExample complete!\n");
    
    return 0;
}
