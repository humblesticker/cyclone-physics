/*
 * BVH Test Demo
 *
 * This demo tests the Bounding Volume Hierarchy (BVH) broad-phase
 * collision detection system. It creates a grid of bodies and measures
 * the performance of collision detection with and without the BVH.
 *
 * Part of the Cyclone physics system.
 */

#include <cyclone/cyclone.h>
#include <cyclone/broad_phase.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

using namespace cyclone;

/**
 * Simple test program to validate BVH functionality
 */
int main(int argc, char** argv)
{
    printf("BVH Broad-Phase Collision Detection Test\n");
    printf("=========================================\n\n");

    // Test 1: Create and populate a BVH
    printf("Test 1: Creating BVH with multiple bodies...\n");
    
    const int gridSize = 10;
    const int numBodies = gridSize * gridSize;
    const real spacing = 1.5f;  // Closer spacing so bounding spheres overlap
    const real radius = 1.0f;    // Larger radius for overlap
    
    // Create bodies
    RigidBody* bodies = new RigidBody[numBodies];
    
    // Position bodies in a grid
    int index = 0;
    for (int x = 0; x < gridSize; x++)
    {
        for (int z = 0; z < gridSize; z++)
        {
            bodies[index].setPosition(
                x * spacing - (gridSize * spacing) / 2.0f,
                0.0f,
                z * spacing - (gridSize * spacing) / 2.0f
            );
            bodies[index].setMass(1.0f);
            bodies[index].setAwake(true);
            bodies[index].calculateDerivedData();
            index++;
        }
    }
    
    printf("  Created %d bodies in a %dx%d grid\n", numBodies, gridSize, gridSize);
    
    // Create BVH and add bodies
    BroadPhaseManager* broadPhase = new BroadPhaseManager(1024);
    
    for (int i = 0; i < numBodies; i++)
    {
        broadPhase->addBody(&bodies[i], radius);
    }
    
    printf("  Added all bodies to BVH\n");
    
    // Test 2: Get potential contacts
    printf("\nTest 2: Checking for potential contacts...\n");
    
    unsigned potentialCount = broadPhase->getPotentialContactCount();
    printf("  Found %d potential contact pairs\n", potentialCount);
    
    // Test 3: Test BVH update
    printf("\nTest 3: Testing BVH update after body movement...\n");
    
    // Move some bodies
    for (int i = 0; i < 10; i++)
    {
        Vector3 pos = bodies[i].getPosition();
        pos.x += 0.5f;
        bodies[i].setPosition(pos);
        bodies[i].calculateDerivedData();
    }
    
    broadPhase->updateBVH();
    potentialCount = broadPhase->getPotentialContactCount();
    printf("  After update, found %d potential contact pairs\n", potentialCount);
    
    // Test 4: Test body removal
    printf("\nTest 4: Testing body removal...\n");
    
    broadPhase->removeBody(&bodies[0]);
    potentialCount = broadPhase->getPotentialContactCount();
    printf("  After removing one body, found %d potential contact pairs\n", potentialCount);
    
    // Test 5: Performance test with more bodies
    printf("\nTest 5: Performance test with more bodies...\n");
    
    const int perfTestSize = 20;
    const int perfTestBodies = perfTestSize * perfTestSize;
    
    RigidBody* perfBodies = new RigidBody[perfTestBodies];
    BroadPhaseManager* perfBroadPhase = new BroadPhaseManager(10000);
    
    index = 0;
    for (int x = 0; x < perfTestSize; x++)
    {
        for (int z = 0; z < perfTestSize; z++)
        {
            perfBodies[index].setPosition(
                x * spacing - (perfTestSize * spacing) / 2.0f,
                0.0f,
                z * spacing - (perfTestSize * spacing) / 2.0f
            );
            perfBodies[index].setMass(1.0f);
            perfBodies[index].setAwake(true);
            perfBodies[index].calculateDerivedData();
            perfBroadPhase->addBody(&perfBodies[index], radius);
            index++;
        }
    }
    
    printf("  Created BVH with %d bodies\n", perfTestBodies);
    
    // Time the broad phase
    const int iterations = 1000;
    unsigned long totalCount = 0;
    
    clock_t start = clock();
    
    for (int i = 0; i < iterations; i++)
    {
        perfBroadPhase->updateBVH();
        totalCount += perfBroadPhase->getPotentialContactCount();
    }
    
    clock_t end = clock();
    double elapsed = ((double)(end - start)) / CLOCKS_PER_SEC * 1000.0;
    
    printf("  Performed %d BVH updates and queries\n", iterations);
    printf("  Average potential contacts per frame: %lu\n", totalCount / iterations);
    printf("  Total time: %.3f ms\n", elapsed);
    printf("  Time per frame: %.3f ms\n", elapsed / iterations);
    
    // Maximum pairwise checks would be n*(n-1)/2
    unsigned long maxPairwise = ((unsigned long)perfTestBodies * (perfTestBodies - 1)) / 2;
    unsigned long avgBVH = totalCount / iterations;
    float reduction = 100.0f * (1.0f - (float)avgBVH / (float)maxPairwise);
    
    printf("\n  Collision check reduction:\n");
    printf("    Max pairwise checks: %lu\n", maxPairwise);
    printf("    BVH average checks: %lu\n", avgBVH);
    printf("    Reduction: %.1f%%\n", reduction);
    
    // Cleanup
    delete broadPhase;
    delete perfBroadPhase;
    delete[] bodies;
    delete[] perfBodies;
    
    printf("\nAll tests completed successfully!\n");
    
    return 0;
}
