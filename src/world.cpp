/*
 * Implementation file for random number generation.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

#include <cstdlib>
#include <cyclone/world.h>

using namespace cyclone;

World::World(unsigned maxContacts, unsigned iterations)
:
firstBody(NULL),
resolver(iterations),
firstContactGen(NULL),
maxContacts(maxContacts)
{
    contacts = new Contact[maxContacts];
    calculateIterations = (iterations == 0);
}

World::~World()
{
    delete[] contacts;
}

void World::startFrame()
{
    BodyRegistration *reg = firstBody;
    while (reg)
    {
        // Remove all forces from the accumulator
        reg->body->clearAccumulators();
        reg->body->calculateDerivedData();

        // Get the next registration
        reg = reg->next;
    }
}

unsigned World::generateContacts()
{
    unsigned limit = maxContacts;
    Contact *nextContact = contacts;

    ContactGenRegistration * reg = firstContactGen;
    while (reg)
    {
        unsigned used = reg->gen->addContact(nextContact, limit);
        limit -= used;
        nextContact += used;

        // We've run out of contacts to fill. This means we're missing
        // contacts.
        if (limit <= 0) break;

        reg = reg->next;
    }

    // Return the number of contacts used.
    return maxContacts - limit;
}

void World::runPhysics(real duration)
{
    // First apply the force generators
    //registry.updateForces(duration);

    // Then integrate the objects
    BodyRegistration *reg = firstBody;
    while (reg)
    {
        // Remove all forces from the accumulator
        reg->body->integrate(duration);

        // Get the next registration
        reg = reg->next;
    }

    // Generate contacts
    unsigned usedContacts = generateContacts();

    // And process them
    if (calculateIterations) resolver.setIterations(usedContacts * 4);
    resolver.resolveContacts(contacts, usedContacts, duration);
}

void World::registerBody(RigidBody* body)
{
    if (!body) return;

    // Create a new registration
    BodyRegistration* reg = new BodyRegistration();
    reg->body = body;
    reg->next = firstBody;
    firstBody = reg;
}

void World::unregisterBody(RigidBody* body)
{
    if (!body || !firstBody) return;

    // Special case: removing the first body
    if (firstBody->body == body)
    {
        BodyRegistration* toDelete = firstBody;
        firstBody = firstBody->next;
        delete toDelete;
        return;
    }

    // Find the body in the list
    BodyRegistration* prev = firstBody;
    BodyRegistration* current = firstBody->next;

    while (current)
    {
        if (current->body == body)
        {
            prev->next = current->next;
            delete current;
            return;
        }
        prev = current;
        current = current->next;
    }
}

void World::registerContactGenerator(ContactGenerator* generator)
{
    if (!generator) return;

    // Create a new registration
    ContactGenRegistration* reg = new ContactGenRegistration();
    reg->gen = generator;
    reg->next = firstContactGen;
    firstContactGen = reg;
}

void World::unregisterContactGenerator(ContactGenerator* generator)
{
    if (!generator || !firstContactGen) return;

    // Special case: removing the first generator
    if (firstContactGen->gen == generator)
    {
        ContactGenRegistration* toDelete = firstContactGen;
        firstContactGen = firstContactGen->next;
        delete toDelete;
        return;
    }

    // Find the generator in the list
    ContactGenRegistration* prev = firstContactGen;
    ContactGenRegistration* current = firstContactGen->next;

    while (current)
    {
        if (current->gen == generator)
        {
            prev->next = current->next;
            delete current;
            return;
        }
        prev = current;
        current = current->next;
    }
}
