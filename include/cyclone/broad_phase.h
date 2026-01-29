/*
 * Interface file for the broad-phase collision detection system.
 *
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

/**
 * @file
 *
 * This file contains the BroadPhaseManager class which uses a Bounding
 * Volume Hierarchy (BVH) to efficiently prune collision candidates
 * before passing them to fine-grained collision detection routines.
 */
#ifndef CYCLONE_BROAD_PHASE_H
#define CYCLONE_BROAD_PHASE_H

#include "collide_coarse.h"
#include "contacts.h"
#include <vector>

namespace cyclone {

    /**
     * The BroadPhaseManager uses a Bounding Volume Hierarchy (BVH)
     * to efficiently identify potential collision pairs. It acts as
     * a ContactGenerator that can be integrated into the World's
     * contact generation pipeline.
     *
     * The manager maintains a dynamic BVH tree that is updated as
     * bodies move, and efficiently culls out pairs of objects that
     * cannot possibly be in contact.
     */
    class BroadPhaseManager : public ContactGenerator
    {
    protected:
        /**
         * The root node of the BVH tree.
         */
        BVHNode<BoundingSphere>* root;

        /**
         * Stores the fine-grained contact generators for potential
         * collision pairs identified by the broad phase.
         */
        std::vector<ContactGenerator*> fineGenerators;

        /**
         * Maximum number of potential contacts to track.
         */
        unsigned maxPotentialContacts;

        /**
         * Array to store potential contacts from BVH traversal.
         */
        PotentialContact* potentialContacts;

    public:
        /**
         * Creates a new BroadPhaseManager with the specified capacity
         * for potential contacts.
         *
         * @param maxPotentialContacts Maximum number of potential contact
         * pairs to identify in the broad phase.
         */
        BroadPhaseManager(unsigned maxPotentialContacts = 1024);

        /**
         * Destructor cleans up the BVH tree and allocated resources.
         */
        ~BroadPhaseManager();

        /**
         * Adds a rigid body to the BVH. The bounding volume is computed
         * from the body's current position and a specified radius.
         *
         * @param body The rigid body to add to the hierarchy.
         * @param radius The radius of the bounding sphere for this body.
         */
        void addBody(RigidBody* body, real radius);

        /**
         * Removes a rigid body from the BVH.
         *
         * @param body The rigid body to remove from the hierarchy.
         */
        void removeBody(RigidBody* body);

        /**
         * Updates the BVH to reflect the current positions of all bodies.
         * This should be called after bodies have been integrated but before
         * contact generation.
         */
        void updateBVH();

        /**
         * Adds a fine-grained contact generator that will be called for
         * potential collision pairs identified by the broad phase.
         *
         * @param generator The fine-grained contact generator to add.
         */
        void addFineGenerator(ContactGenerator* generator);

        /**
         * Implementation of ContactGenerator interface. Identifies potential
         * collision pairs using the BVH and generates contacts using the
         * registered fine-grained contact generators.
         *
         * @param contact Pointer to the contact array to fill.
         * @param limit Maximum number of contacts to generate.
         * @return The number of contacts generated.
         */
        virtual unsigned addContact(Contact* contact, unsigned limit) const;

        /**
         * Gets the number of potential collision pairs identified by the
         * broad phase in the last contact generation pass.
         *
         * @return The number of potential collision pairs.
         */
        unsigned getPotentialContactCount() const;

    protected:
        /**
         * Rebuilds the entire BVH from scratch. This is used when bodies
         * are added or removed, or when the tree becomes too unbalanced.
         */
        void rebuildBVH();

        /**
         * Creates a bounding sphere for a rigid body.
         *
         * @param body The rigid body.
         * @param radius The radius of the bounding sphere.
         * @return A BoundingSphere centered at the body's position.
         */
        BoundingSphere createBoundingSphere(RigidBody* body, real radius) const;

        /**
         * Structure to track bodies and their bounding sphere radii.
         */
        struct BodyRecord
        {
            RigidBody* body;
            real radius;
        };

        /**
         * List of all bodies managed by this broad phase.
         */
        std::vector<BodyRecord> bodies;
    };

} // namespace cyclone

#endif // CYCLONE_BROAD_PHASE_H
