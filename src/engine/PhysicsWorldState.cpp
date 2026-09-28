/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2024 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

/*
    (local fix) Determinism support: rebuildInternalState() and computeStateHash().

    WHY. A world's result depends not only on the bodies in it but on the order its internals
    were built in. The same bodies, added to a world that has held others, are solved in a
    different order and the last bits of the result differ. Measured (2026-09-28, a harness shaped
    like archer's restart): the decisive carry-over is the BROAD PHASE. Tree node ids come off a
    LIFO free list, so a recreated collider gets a recycled id; overlapping pair ids are built from
    those ids, which collider of a pair is "1" follows the tree's report order, and the order
    pairs are found in becomes the order contacts are solved in. Resetting only the contacts or
    only the component order changed nothing; rebuilding the tree made every pass identical.

    THE CONTRACT of rebuildInternalState(): everything observable through the public API is kept
    (transforms, velocities, the sleeping flag, every setting, every pointer); the hidden history is
    set to what a newly built world would hold (the tree and its ids, pairs, contacts, component
    order, sleep timers, warm-start impulses). Call it between two update()s, after the bodies are
    where they should be.
*/

// Libraries
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/body/RigidBody.h>
#include <reactphysics3d/collision/Collider.h>
#include <reactphysics3d/collision/shapes/CollisionShape.h>
#include <reactphysics3d/constraint/VehicleConstraint.h>
#include <reactphysics3d/constraint/UprightConstraint.h>
#include <reactphysics3d/containers/Set.h>
#include <reactphysics3d/containers/Map.h>
#include <algorithm>
#include <cstring>

using namespace reactphysics3d;

namespace {

// FNV-1a 64 over raw bytes. Values are fed field by field, never as structs, so padding bytes
// (whatever the allocator left in them) can never reach the hash.
struct Fnv {

    uint64 h = 14695981039346656037ull;

    void bytes(const void* data, size_t n) {
        const unsigned char* b = static_cast<const unsigned char*>(data);
        for (size_t i = 0; i < n; i++) {
            h ^= b[i];
            h *= 1099511628211ull;
        }
    }
    void u8(uint8 v) { bytes(&v, 1); }
    void b(bool v) { u8(v ? 1 : 0); }
    void u32(uint32 v) { bytes(&v, sizeof(v)); }
    void i32(int32 v) { bytes(&v, sizeof(v)); }
    void u64(uint64 v) { bytes(&v, sizeof(v)); }
    void f(decimal v) { bytes(&v, sizeof(v)); }
    void v2(const Vector2& v) { f(v.x); f(v.y); }
    void v3(const Vector3& v) { f(v.x); f(v.y); f(v.z); }
    void q(const Quaternion& v) { f(v.x); f(v.y); f(v.z); f(v.w); }
    void tr(const Transform& t) { v3(t.getPosition()); q(t.getOrientation()); }
    void aabb(const AABB& a) { v3(a.getMin()); v3(a.getMax()); }
    void m2(const Matrix2x2& m) { for (int r = 0; r < 2; r++) v2(m[r]); }
    void m3(const Matrix3x3& m) { for (int r = 0; r < 3; r++) v3(m[r]); }
};

const uint32 NONE = 0xFFFFFFFFu;

}

// The canonical order: bodies in creation order, their colliders in the order they were added,
// joints by the first body that lists them. Every list involved only appends and removes in
// order, so for the objects alive now these are the orders they were created in.
void PhysicsWorld::computeCanonicalOrder(Array<Entity>& bodies, Array<Entity>& colliders, Array<Entity>& joints) const {

    MemoryAllocator& allocator = mMemoryManager.getHeapAllocator();

    for (uint32 b = 0; b < mRigidBodies.size(); b++) {
        bodies.add(mRigidBodies[b]->getEntity());
    }
    for (uint32 b = 0; b < bodies.size(); b++) {
        const Array<Entity>& bodyColliders = mBodyComponents.getColliders(bodies[b]);
        for (uint32 c = 0; c < bodyColliders.size(); c++) {
            colliders.add(bodyColliders[c]);
        }
    }
    Set<Entity> seenJoints(allocator);
    for (uint32 b = 0; b < bodies.size(); b++) {
        const Array<Entity>& bodyJoints = mRigidBodyComponents.getJoints(bodies[b]);
        for (uint32 j = 0; j < bodyJoints.size(); j++) {
            if (!seenJoints.contains(bodyJoints[j])) {
                seenJoints.add(bodyJoints[j]);
                joints.add(bodyJoints[j]);
            }
        }
    }
}

// Leave the world internally as if it had just been built from its bodies and joints
void PhysicsWorld::rebuildInternalState() {

    MemoryAllocator& allocator = mMemoryManager.getHeapAllocator();

    Array<Entity> bodies(allocator), colliders(allocator), joints(allocator);
    computeCanonicalOrder(bodies, colliders, joints);

    // 1. Everything out of the broad-phase. Removing a collider also removes every overlapping
    //    pair it is in, with the pair's last-frame collision info (cached SAT axes and the like).
    for (uint32 c = 0; c < colliders.size(); c++) {
        Collider* collider = mCollidersComponents.getCollider(colliders[c]);
        if (collider->getBroadPhaseId() != -1) {
            mCollisionDetection.removeCollider(collider);
        }
    }
    mCollisionDetection.mBroadPhaseSystem.resetToFresh();

    // 2. The contacts. The next frame warm-starts from last frame's, found by pair id - and pair
    //    ids are built from broad-phase ids, which are recycled, so a new pair could otherwise
    //    inherit the impulses of a destroyed one that happened to get the same ids.
    mCollisionDetection.mContactPairs1.clear();
    mCollisionDetection.mContactPairs2.clear();
    mCollisionDetection.mContactManifolds1.clear();
    mCollisionDetection.mContactManifolds2.clear();
    mCollisionDetection.mContactPoints1.clear();
    mCollisionDetection.mContactPoints2.clear();
    mCollisionDetection.mPreviousMapPairIdToContactPairIndex.clear();
    mCollisionDetection.mLostContactPairs.clear();

    // 3. The component arrays in canonical order. Removal swaps the last element into the gap and
    //    sleeping moves components across the enabled/disabled boundary, so their order is history.
    //    Islands are built by walking these arrays.
    mBodyComponents.setEntitiesOrder(bodies);
    mRigidBodyComponents.setEntitiesOrder(bodies);
    mTransformComponents.setEntitiesOrder(bodies);
    mCollidersComponents.setEntitiesOrder(colliders);
    mJointsComponents.setEntitiesOrder(joints);

    Array<Entity> ofType(allocator);
    const JointType types[5] = {JointType::BALLSOCKETJOINT, JointType::FIXEDJOINT, JointType::HINGEJOINT,
                                JointType::SLIDERJOINT, JointType::SPRINGJOINT};
    Components* typeComponents[5] = {&mBallAndSocketJointsComponents, &mFixedJointsComponents,
                                     &mHingeJointsComponents, &mSliderJointsComponents, &mSpringJointsComponents};
    for (uint32 t = 0; t < 5; t++) {
        ofType.clear();
        for (uint32 j = 0; j < joints.size(); j++) {
            if (mJointsComponents.mTypes[mJointsComponents.getEntityIndex(joints[j])] == types[t]) {
                ofType.add(joints[j]);
            }
        }
        typeComponents[t]->setEntitiesOrder(ofType);
    }

    // 4. The hidden accumulators, to what a new body or joint starts with.
    //    Sleep timers: waking an awake body is a no-op, so nothing public ever resets one.
    for (uint32 b = 0; b < bodies.size(); b++) {
        mRigidBodyComponents.setSleepTime(bodies[b], decimal(0.0));
    }
    //    Warm-start impulses of joints and of the vehicle and upright constraints.
    for (uint32 j = 0; j < joints.size(); j++) {
        const Entity joint = joints[j];
        switch (mJointsComponents.mTypes[mJointsComponents.getEntityIndex(joint)]) {
            case JointType::BALLSOCKETJOINT:
                mBallAndSocketJointsComponents.setImpulse(joint, Vector3::zero());
                mBallAndSocketJointsComponents.setConeLimitImpulse(joint, decimal(0.0));
                break;
            case JointType::FIXEDJOINT:
                mFixedJointsComponents.setImpulseTranslation(joint, Vector3::zero());
                mFixedJointsComponents.setImpulseRotation(joint, Vector3::zero());
                break;
            case JointType::HINGEJOINT:
                mHingeJointsComponents.setImpulseTranslation(joint, Vector3::zero());
                mHingeJointsComponents.setImpulseRotation(joint, Vector2::zero());
                mHingeJointsComponents.setImpulseLowerLimit(joint, decimal(0.0));
                mHingeJointsComponents.setImpulseUpperLimit(joint, decimal(0.0));
                mHingeJointsComponents.setImpulseMotor(joint, decimal(0.0));
                break;
            case JointType::SLIDERJOINT:
                mSliderJointsComponents.setImpulseTranslation(joint, Vector2::zero());
                mSliderJointsComponents.setImpulseRotation(joint, Vector3::zero());
                mSliderJointsComponents.setImpulseLowerLimit(joint, decimal(0.0));
                mSliderJointsComponents.setImpulseUpperLimit(joint, decimal(0.0));
                mSliderJointsComponents.setImpulseMotor(joint, decimal(0.0));
                break;
            case JointType::SPRINGJOINT:
                mSpringJointsComponents.mConstraintPart[mSpringJointsComponents.getEntityIndex(joint)].resetTotalLambda();
                break;
        }
    }
    for (uint32 v = 0; v < mVehicles.size(); v++) {
        for (uint32 w = 0; w < mVehicles[v]->mWheels.size(); w++) {
            VehicleWheel& wheel = mVehicles[v]->mWheels[w];
            wheel.mSuspensionPart.resetTotalLambda();
            wheel.mHardStopPart.resetTotalLambda();
            wheel.mLongitudinalPart.resetTotalLambda();
            wheel.mLateralPart.resetTotalLambda();
        }
    }
    for (uint32 u = 0; u < mUprightConstraints.size(); u++) {
        mUprightConstraints[u]->mPart.resetTotalLambda();
        mUprightConstraints[u]->mSpinDampingPart.resetTotalLambda();
    }

    // 5. Back into a fresh tree in canonical order, as a new world's bodies would go in. Each
    //    goes in as a moved shape, so the next update() finds every pair again, in that order.
    //    A pair of two SLEEPING bodies comes back enabled - as for bodies created and then put to
    //    sleep - so its contacts are computed (not solved: islands start from awake bodies).
    for (uint32 c = 0; c < colliders.size(); c++) {
        const Entity bodyEntity = mCollidersComponents.getBody(colliders[c]);
        if (!mBodyComponents.getIsActive(bodyEntity)) continue;
        Collider* collider = mCollidersComponents.getCollider(colliders[c]);
        const Transform& bodyTransform = mTransformComponents.getTransform(bodyEntity);
        const AABB aabb = collider->getCollisionShape()->computeTransformedAABB(
                    bodyTransform * mCollidersComponents.getLocalToBodyTransform(colliders[c]));
        mCollisionDetection.addCollider(collider, aabb);
    }
}

// Hash everything the next update() depends on
PhysicsWorld::StateHash PhysicsWorld::computeStateHash() const {

    MemoryAllocator& allocator = mMemoryManager.getHeapAllocator();

    Array<Entity> bodies(allocator), colliders(allocator), joints(allocator);
    computeCanonicalOrder(bodies, colliders, joints);

    // Objects are named by their canonical index, never by entity id or pointer: ids and
    // addresses are history, and differ between two runs that are otherwise the same.
    Map<Entity, uint32> bodyIndex(allocator), colliderIndex(allocator), jointIndex(allocator);
    for (uint32 i = 0; i < bodies.size(); i++) bodyIndex.add(Pair<Entity, uint32>(bodies[i], i));
    for (uint32 i = 0; i < colliders.size(); i++) colliderIndex.add(Pair<Entity, uint32>(colliders[i], i));
    for (uint32 i = 0; i < joints.size(); i++) jointIndex.add(Pair<Entity, uint32>(joints[i], i));
    auto bodyOf = [&](Entity e) { auto it = bodyIndex.find(e); return it == bodyIndex.end() ? NONE : it->second; };
    auto colliderOf = [&](Entity e) { auto it = colliderIndex.find(e); return it == colliderIndex.end() ? NONE : it->second; };

    StateHash out;

    // ---- world: the settings the step reads ----
    {
        Fnv h;
        h.v3(mConfig.gravity);
        h.f(mConfig.persistentContactDistanceThreshold);
        h.f(mConfig.defaultFrictionCoefficient);
        h.f(mConfig.defaultBounciness);
        h.f(mConfig.restitutionVelocityThreshold);
        h.f(mConfig.cosAngleSimilarContactManifold);
        h.b(mIsGravityEnabled);
        h.u32(mNbVelocitySolverIterations);
        h.u32(mNbPositionSolverIterations);
        h.b(mIsSleepingEnabled);
        h.f(mSleepLinearVelocity);
        h.f(mSleepAngularVelocity);
        h.f(mTimeBeforeSleep);
        h.b(mContactSolverSystem.isSplitImpulseActive());
        out.world = h.h;
    }

    // ---- bodies ----
    {
        Fnv h;
        h.u32(bodies.size());
        for (uint32 i = 0; i < bodies.size(); i++) {
            const Entity e = bodies[i];
            const uint32 k = mRigidBodyComponents.getEntityIndex(e);
            h.u32(static_cast<uint32>(mRigidBodyComponents.mBodyTypes[k]));
            h.b(mBodyComponents.getIsActive(e));
            h.b(mRigidBodyComponents.mIsAllowedToSleep[k]);
            h.b(mRigidBodyComponents.mIsSleeping[k]);
            h.f(mRigidBodyComponents.mSleepTimes[k]);
            h.b(mRigidBodyComponents.mIsGravityEnabled[k]);
            h.tr(mTransformComponents.getTransform(e));
            h.v3(mRigidBodyComponents.mLinearVelocities[k]);
            h.v3(mRigidBodyComponents.mAngularVelocities[k]);
            h.v3(mRigidBodyComponents.mExternalForces[k]);
            h.v3(mRigidBodyComponents.mExternalTorques[k]);
            h.f(mRigidBodyComponents.mLinearDampings[k]);
            h.f(mRigidBodyComponents.mAngularDampings[k]);
            h.f(mRigidBodyComponents.mMasses[k]);
            h.f(mRigidBodyComponents.mInverseMasses[k]);
            h.v3(mRigidBodyComponents.mLocalInertiaTensors[k]);
            h.v3(mRigidBodyComponents.mInverseInertiaTensorsLocal[k]);
            h.v3(mRigidBodyComponents.mCentersOfMassLocal[k]);
            h.v3(mRigidBodyComponents.mCentersOfMassWorld[k]);
            h.v3(mRigidBodyComponents.mLinearLockAxisFactors[k]);
            h.v3(mRigidBodyComponents.mAngularLockAxisFactors[k]);
            h.u32(mBodyComponents.getColliders(e).size());
            h.u32(mRigidBodyComponents.mJoints[k].size());
        }
        out.bodies = h.h;
    }

    // ---- colliders ----
    {
        Fnv h;
        h.u32(colliders.size());
        for (uint32 i = 0; i < colliders.size(); i++) {
            const Entity e = colliders[i];
            h.u32(bodyOf(mCollidersComponents.getBody(e)));
            const CollisionShape* shape = mCollidersComponents.getCollisionShape(e);
            h.u32(static_cast<uint32>(shape->getName()));
            h.u32(static_cast<uint32>(shape->getType()));
            h.aabb(shape->getLocalBounds());
            h.tr(mCollidersComponents.getLocalToBodyTransform(e));
            h.tr(mCollidersComponents.getLocalToWorldTransform(e));
            const Material& material = mCollidersComponents.getCollider(e)->getMaterial();
            h.f(material.getFrictionCoefficientSqrt());
            h.f(material.getBounciness());
            h.f(material.getMassDensity());
            h.u32(mCollidersComponents.getCollisionCategoryBits(e));
            h.u32(mCollidersComponents.getCollideWithMaskBits(e));
            h.b(mCollidersComponents.getIsTrigger(e));
            h.b(mCollidersComponents.getIsSimulationCollider(e));
            h.b(mCollidersComponents.getIsWorldQueryCollider(e));
            h.i32(mCollidersComponents.getBroadPhaseId(e));
        }
        out.colliders = h.h;
    }

    // ---- order: where each object sits in every array the step walks ----
    {
        Fnv h;
        auto layout = [&](const Components& components, const Array<Entity>& entities) {
            h.u32(components.getNbComponents());
            h.u32(components.getNbEnabledComponents());
            for (uint32 i = 0; i < entities.size(); i++) h.u32(components.getEntityIndex(entities[i]));
        };
        layout(mBodyComponents, bodies);
        layout(mRigidBodyComponents, bodies);
        layout(mTransformComponents, bodies);
        layout(mCollidersComponents, colliders);
        layout(mJointsComponents, joints);
        const JointType types[5] = {JointType::BALLSOCKETJOINT, JointType::FIXEDJOINT, JointType::HINGEJOINT,
                                    JointType::SLIDERJOINT, JointType::SPRINGJOINT};
        const Components* typeComponents[5] = {&mBallAndSocketJointsComponents, &mFixedJointsComponents,
                                               &mHingeJointsComponents, &mSliderJointsComponents, &mSpringJointsComponents};
        Array<Entity> ofType(allocator);
        for (uint32 t = 0; t < 5; t++) {
            ofType.clear();
            for (uint32 j = 0; j < joints.size(); j++) {
                if (mJointsComponents.mTypes[mJointsComponents.getEntityIndex(joints[j])] == types[t]) ofType.add(joints[j]);
            }
            layout(*typeComponents[t], ofType);
        }
        out.order = h.h;
    }

    // ---- broadphase: the tree node by node, the free list, and the moved shapes ----
    {
        Fnv h;
        const DynamicAABBTree& tree = mCollisionDetection.mBroadPhaseSystem.mDynamicAABBTree;
        h.i32(tree.mRootNodeID);
        h.i32(tree.mFreeNodeID);
        h.i32(tree.mNbNodes);
        h.i32(tree.mNbAllocatedNodes);
        h.f(tree.mFatAABBInflatePercentage);
        for (int32 n = 0; n < tree.mNbAllocatedNodes; n++) {
            const TreeNode& node = tree.mNodes[n];
            h.i32(node.height);
            if (node.height < 0) {
                // A free node: its link is the free list, which decides every id handed out next
                h.i32(node.nextNodeID);
                continue;
            }
            h.i32(node.parentID);
            h.aabb(node.aabb);
            if (node.height == 0) {
                auto it = mCollisionDetection.mMapBroadPhaseIdToColliderEntity.find(n);
                h.u32(it == mCollisionDetection.mMapBroadPhaseIdToColliderEntity.end() ? NONE : colliderOf(it->second));
            }
            else {
                h.i32(node.children[0]);
                h.i32(node.children[1]);
            }
        }
        // Walked by bucket, so the bucket count - the most it has ever held - changes the order
        const Set<int>& moved = mCollisionDetection.mBroadPhaseSystem.mMovedShapes;
        h.u64(moved.size());
        h.u64(moved.capacity());
        for (auto it = moved.begin(); it != moved.end(); ++it) h.i32(*it);
        out.broadphase = h.h;
    }

    // ---- pairs ----
    {
        Fnv h;
        auto lastFrame = [&](const LastFrameCollisionInfo& info) {
            h.b(info.isValid); h.b(info.isObsolete); h.b(info.wasColliding);
            h.b(info.wasUsingGJK); h.b(info.wasUsingSAT); h.v3(info.gjkSeparatingAxis);
            h.b(info.satIsAxisFacePolyhedron1); h.b(info.satIsAxisFacePolyhedron2);
            h.u8(info.satMinAxisFaceIndex); h.u8(info.satMinEdge1Index); h.u8(info.satMinEdge2Index);
        };
        auto pairHeader = [&](const OverlappingPairs::OverlappingPair& pair) {
            h.u64(pair.pairID);
            h.i32(pair.broadPhaseId1); h.i32(pair.broadPhaseId2);
            h.u32(colliderOf(pair.collider1)); h.u32(colliderOf(pair.collider2));
            h.b(pair.needToTestOverlap);
            h.u32(static_cast<uint32>(pair.narrowPhaseAlgorithmType));
            h.b(pair.collidingInPreviousFrame); h.b(pair.collidingInCurrentFrame);
            h.b(pair.isEnabled);
        };
        const OverlappingPairs& pairs = mCollisionDetection.mOverlappingPairs;
        h.u64(pairs.mConvexPairs.size());
        for (uint64 p = 0; p < pairs.mConvexPairs.size(); p++) {
            pairHeader(pairs.mConvexPairs[p]);
            lastFrame(pairs.mConvexPairs[p].lastFrameCollisionInfo);
        }
        h.u64(pairs.mConcavePairs.size());
        Array<uint64> keys(allocator);
        for (uint64 p = 0; p < pairs.mConcavePairs.size(); p++) {
            const OverlappingPairs::ConcaveOverlappingPair& pair = pairs.mConcavePairs[p];
            pairHeader(pair);
            h.b(pair.isShape1Convex);
            // Looked up by triangle, so what counts is the content: hash it in key order
            keys.clear();
            for (auto it = pair.lastFrameCollisionInfos.begin(); it != pair.lastFrameCollisionInfos.end(); ++it) keys.add(it->first);
            if (keys.size() > 0) std::sort(&keys[0], &keys[0] + keys.size());
            h.u64(keys.size());
            for (uint32 k = 0; k < keys.size(); k++) {
                h.u64(keys[k]);
                lastFrame(*pair.lastFrameCollisionInfos[keys[k]]);
            }
        }
        // Body pairs that may not collide (joints with collision off), by canonical index
        Array<uint64> noCollision(allocator);
        for (auto it = mCollisionDetection.mNoCollisionPairs.begin(); it != mCollisionDetection.mNoCollisionPairs.end(); ++it) {
            const uint32 x = bodyOf(it->first);
            const uint32 y = bodyOf(it->second);
            noCollision.add((static_cast<uint64>(std::min(x, y)) << 32) | std::max(x, y));
        }
        if (noCollision.size() > 0) std::sort(&noCollision[0], &noCollision[0] + noCollision.size());
        h.u64(noCollision.size());
        for (uint32 k = 0; k < noCollision.size(); k++) h.u64(noCollision[k]);
        out.pairs = h.h;
    }

    // ---- contacts: the ones the next step warm-starts from ----
    {
        Fnv h;
        const Array<ContactPair>& contactPairs = *mCollisionDetection.mCurrentContactPairs;
        const Array<ContactManifold>& manifolds = *mCollisionDetection.mCurrentContactManifolds;
        const Array<ContactPoint>& points = *mCollisionDetection.mCurrentContactPoints;
        h.u64(contactPairs.size());
        for (uint64 p = 0; p < contactPairs.size(); p++) {
            const ContactPair& pair = contactPairs[p];
            h.u64(pair.pairId);
            h.u32(bodyOf(pair.body1Entity)); h.u32(bodyOf(pair.body2Entity));
            h.u32(colliderOf(pair.collider1Entity)); h.u32(colliderOf(pair.collider2Entity));
            h.b(pair.isTrigger); h.b(pair.collidingInPreviousFrame);
            h.u32(pair.nbContactManifolds); h.u32(pair.nbToTalContactPoints);
            for (uint32 m = pair.contactManifoldsIndex; m < pair.contactManifoldsIndex + pair.nbContactManifolds; m++) {
                const ContactManifold& manifold = manifolds[m];
                h.u8(manifold.nbContactPoints);
                h.v3(manifold.frictionVector1); h.v3(manifold.frictionVector2);
                h.f(manifold.frictionImpulse1); h.f(manifold.frictionImpulse2);
                h.f(manifold.frictionTwistImpulse);
            }
            for (uint32 c = pair.contactPointsIndex; c < pair.contactPointsIndex + pair.nbToTalContactPoints; c++) {
                const ContactPoint& point = points[c];
                h.v3(point.getNormal());
                h.v3(point.getLocalPointOnShape1()); h.v3(point.getLocalPointOnShape2());
                h.f(point.getPenetrationDepth()); h.f(point.getPenetrationImpulse());
                h.b(point.getIsRestingContact());
            }
        }
        out.contacts = h.h;
    }

    // ---- joints: parameters and warm-start impulses (per-step scratch is recomputed, left out) ----
    {
        Fnv h;
        h.u32(joints.size());
        for (uint32 j = 0; j < joints.size(); j++) {
            const Entity e = joints[j];
            const uint32 k = mJointsComponents.getEntityIndex(e);
            h.u32(static_cast<uint32>(mJointsComponents.mTypes[k]));
            h.u32(bodyOf(mJointsComponents.mBody1Entities[k]));
            h.u32(bodyOf(mJointsComponents.mBody2Entities[k]));
            h.b(mJointsComponents.mIsCollisionEnabled[k]);
            h.u32(static_cast<uint32>(mJointsComponents.mPositionCorrectionTechniques[k]));
            switch (mJointsComponents.mTypes[k]) {
                case JointType::BALLSOCKETJOINT: {
                    BallAndSocketJointComponents& c = const_cast<BallAndSocketJointComponents&>(mBallAndSocketJointsComponents);   // some getters are not const; only read
                    h.v3(c.getLocalAnchorPointBody1(e)); h.v3(c.getLocalAnchorPointBody2(e));
                    h.v3(c.getImpulse(e));
                    h.b(c.getIsConeLimitEnabled(e)); h.f(c.getConeLimitHalfAngle(e)); h.f(c.getConeLimitImpulse(e));
                    break;
                }
                case JointType::FIXEDJOINT: {
                    FixedJointComponents& c = const_cast<FixedJointComponents&>(mFixedJointsComponents);   // some getters are not const; only read
                    h.v3(c.getLocalAnchorPointBody1(e)); h.v3(c.getLocalAnchorPointBody2(e));
                    h.v3(c.getImpulseTranslation(e)); h.v3(c.getImpulseRotation(e));
                    h.q(c.getInitOrientationDifferenceInv(e));
                    break;
                }
                case JointType::HINGEJOINT: {
                    HingeJointComponents& c = const_cast<HingeJointComponents&>(mHingeJointsComponents);   // some getters are not const; only read
                    h.v3(c.getLocalAnchorPointBody1(e)); h.v3(c.getLocalAnchorPointBody2(e));
                    h.v3(c.getHingeLocalAxisBody1(e)); h.v3(c.getHingeLocalAxisBody2(e));
                    h.q(c.getInitOrientationDifferenceInv(e));
                    h.v3(c.getImpulseTranslation(e)); h.v2(c.getImpulseRotation(e));
                    h.f(c.getImpulseLowerLimit(e)); h.f(c.getImpulseUpperLimit(e)); h.f(c.getImpulseMotor(e));
                    h.b(c.getIsLimitEnabled(e)); h.b(c.getIsMotorEnabled(e));
                    h.f(c.getLowerLimit(e)); h.f(c.getUpperLimit(e));
                    h.f(c.getMotorSpeed(e)); h.f(c.getMaxMotorTorque(e));
                    break;
                }
                case JointType::SLIDERJOINT: {
                    SliderJointComponents& c = const_cast<SliderJointComponents&>(mSliderJointsComponents);   // some getters are not const; only read
                    h.v3(c.getLocalAnchorPointBody1(e)); h.v3(c.getLocalAnchorPointBody2(e));
                    h.v3(c.getSliderAxisBody1(e));
                    h.q(c.getInitOrientationDifferenceInv(e));
                    h.v2(c.getImpulseTranslation(e)); h.v3(c.getImpulseRotation(e));
                    h.f(c.getImpulseLowerLimit(e)); h.f(c.getImpulseUpperLimit(e)); h.f(c.getImpulseMotor(e));
                    h.b(c.getIsLimitEnabled(e)); h.b(c.getIsMotorEnabled(e));
                    h.f(c.getLowerLimit(e)); h.f(c.getUpperLimit(e));
                    h.f(c.getMotorSpeed(e)); h.f(c.getMaxMotorForce(e));
                    break;
                }
                case JointType::SPRINGJOINT: {
                    const SpringJointComponents& c = mSpringJointsComponents;
                    h.v3(c.getLocalAnchorPointBody1(e)); h.v3(c.getLocalAnchorPointBody2(e));
                    h.f(c.getRestLength(e));
                    const SpringSettings& s = c.getSpringSettings(e);
                    h.u32(static_cast<uint32>(s.mode)); h.f(s.frequencyOrStiffness); h.f(s.damping);
                    h.f(c.getConstraintPart(e).getTotalLambda());
                    break;
                }
            }
        }
        out.joints = h.h;
    }

    // ---- constraints: vehicles and upright constraints, in the order they were created ----
    {
        Fnv h;
        h.u32(mVehicles.size());
        for (uint32 v = 0; v < mVehicles.size(); v++) {
            const VehicleConstraint& vehicle = *mVehicles[v];
            h.u32(bodyOf(vehicle.mBody->getEntity()));
            h.v3(vehicle.mUp); h.v3(vehicle.mForward);
            h.f(vehicle.mCosMaxSlopeAngle); h.u32(vehicle.mRaycastCategoryMaskBits);
            h.f(vehicle.mWarmStartImpulseRatio);
            h.u32(vehicle.mWheels.size());
            for (uint32 w = 0; w < vehicle.mWheels.size(); w++) {
                const VehicleWheel& wheel = vehicle.mWheels[w];
                h.f(wheel.mSteerAngle); h.f(wheel.mDriveTorque); h.f(wheel.mBrakeTorque); h.f(wheel.mBrakeImpulse);
                h.u32(wheel.mContactBody != nullptr ? bodyOf(wheel.mContactBody->getEntity()) : NONE);
                h.b(wheel.mIsContactBodyFixed);
                h.v3(wheel.mContactPoint); h.v3(wheel.mContactNormal);
                h.v3(wheel.mContactLongitudinal); h.v3(wheel.mContactLateral);
                h.f(wheel.mAxlePlaneConstant); h.f(wheel.mSuspensionLength);
                h.f(wheel.mLateralSlipAngle); h.f(wheel.mCombinedSlip); h.f(wheel.mGripScale);
                h.f(wheel.mSlidingFraction); h.f(wheel.mLongitudinalGripShare); h.f(wheel.mLateralGripShare);
                h.f(wheel.mAngularVelocity); h.f(wheel.mRotationAngle);
                h.f(wheel.mSuspensionPart.getTotalLambda()); h.f(wheel.mHardStopPart.getTotalLambda());
                h.f(wheel.mLongitudinalPart.getTotalLambda()); h.f(wheel.mLateralPart.getTotalLambda());
            }
        }
        h.u32(mUprightConstraints.size());
        for (uint32 u = 0; u < mUprightConstraints.size(); u++) {
            const UprightConstraint& upright = *mUprightConstraints[u];
            h.u32(bodyOf(upright.mBody->getEntity()));
            h.f(upright.mCosMaxAngle); h.f(upright.mCurrentAngle);
            h.v3(upright.mRotationAxis); h.v3(upright.mSpinAxis);
            h.f(upright.mPart.getTotalLambda()); h.f(upright.mSpinDampingPart.getTotalLambda());
        }
        out.constraints = h.h;
    }

    Fnv total;
    const uint64 parts[9] = {out.world, out.bodies, out.colliders, out.order, out.broadphase,
                             out.pairs, out.contacts, out.joints, out.constraints};
    total.bytes(parts, sizeof(parts));
    out.total = total.h;
    return out;
}
