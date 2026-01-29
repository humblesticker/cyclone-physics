/*
 * Implementation file for the broad-phase collision detection system.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

#include <cyclone/broad_phase.h>
#include <algorithm>

using namespace cyclone;

BroadPhaseManager::BroadPhaseManager(unsigned maxPotentialContacts)
    : root(NULL), maxPotentialContacts(maxPotentialContacts)
{
    potentialContacts = new PotentialContact[maxPotentialContacts];
}

BroadPhaseManager::~BroadPhaseManager()
{
    delete[] potentialContacts;
    if (root)
    {
        delete root;
        root = NULL;
    }
}

void BroadPhaseManager::addBody(RigidBody* body, real radius)
{
    if (!body) return;

    // Create a body record
    BodyRecord record;
    record.body = body;
    record.radius = radius;
    bodies.push_back(record);

    // Create bounding sphere for the body
    BoundingSphere sphere = createBoundingSphere(body, radius);

    // Insert into BVH
    if (!root)
    {
        // Create root node if this is the first body
        root = new BVHNode<BoundingSphere>(NULL, sphere, body);
    }
    else
    {
        // Insert into existing tree
        root->insert(body, sphere);
    }
}

void BroadPhaseManager::removeBody(RigidBody* body)
{
    if (!body) return;

    // Find and remove the body from our list
    for (auto it = bodies.begin(); it != bodies.end(); ++it)
    {
        if (it->body == body)
        {
            bodies.erase(it);
            break;
        }
    }

    // Rebuild the BVH since removal is complex
    rebuildBVH();
}

void BroadPhaseManager::updateBVH()
{
    // For dynamic scenes, we rebuild the BVH each frame
    // This is simpler and often faster than incremental updates
    // for highly dynamic scenes with many moving objects
    if (root && !bodies.empty())
    {
        rebuildBVH();
    }
}

void BroadPhaseManager::addFineGenerator(ContactGenerator* generator)
{
    if (generator)
    {
        fineGenerators.push_back(generator);
    }
}

unsigned BroadPhaseManager::addContact(Contact* contact, unsigned limit) const
{
    if (!root || limit == 0)
    {
        return 0;
    }

    // Get potential contacts from the BVH
    unsigned potentialCount = root->getPotentialContacts(
        potentialContacts, 
        std::min(limit, maxPotentialContacts)
    );

    // For now, we don't have fine-grained generators set up,
    // so we return 0 contacts. The integration with fine-grained
    // detection would happen here.
    // In a full implementation, we would iterate through potential
    // contacts and call fine-grained collision detection for each pair.
    
    // This allows the broad phase to be used for culling even without
    // fine-grained generators, with the contact generation being handled
    // by other means.
    
    unsigned contactsGenerated = 0;
    
    // If we have fine-grained generators, use them
    for (const auto& generator : fineGenerators)
    {
        if (limit - contactsGenerated <= 0) break;
        
        unsigned used = generator->addContact(
            contact + contactsGenerated, 
            limit - contactsGenerated
        );
        contactsGenerated += used;
    }

    return contactsGenerated;
}

unsigned BroadPhaseManager::getPotentialContactCount() const
{
    if (!root)
    {
        return 0;
    }

    return root->getPotentialContacts(potentialContacts, maxPotentialContacts);
}

void BroadPhaseManager::rebuildBVH()
{
    // Delete the old tree
    if (root)
    {
        delete root;
        root = NULL;
    }

    // If no bodies, nothing to do
    if (bodies.empty())
    {
        return;
    }

    // Build new tree from scratch
    // Start with the first body as the root
    BoundingSphere sphere = createBoundingSphere(bodies[0].body, bodies[0].radius);
    root = new BVHNode<BoundingSphere>(NULL, sphere, bodies[0].body);

    // Insert remaining bodies
    for (size_t i = 1; i < bodies.size(); ++i)
    {
        sphere = createBoundingSphere(bodies[i].body, bodies[i].radius);
        root->insert(bodies[i].body, sphere);
    }
}

BoundingSphere BroadPhaseManager::createBoundingSphere(RigidBody* body, real radius) const
{
    Vector3 position = body->getPosition();
    return BoundingSphere(position, radius);
}
