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

#ifndef TEST_WORLD_DETERMINISM_H
#define TEST_WORLD_DETERMINISM_H

// (local fix) PhysicsWorld::rebuildInternalState() and PhysicsWorld::computeStateHash().
//
// The situation they are for: an app restarts a game by destroying some bodies and creating them
// again in the same place, while others (terrain, the player) survive. The world is then not the
// one a fresh start would have built - its broad-phase hands the new colliders recycled ids, and
// everything ordered by them comes out in a different order - so a replay after the restart
// drifts from the same replay after a fresh start. See src/engine/PhysicsWorldState.cpp.

// Libraries
#include <reactphysics3d/reactphysics3d.h>
#include <vector>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestWorldDeterminism
class TestWorldDeterminism : public Test {

    private :

        // ---------- Atributes ---------- //

        PhysicsCommon mPhysicsCommon;

        BoxShape* mGroundShape;
        BoxShape* mCrateShape;
        BoxShape* mDartShape;
        SphereShape* mLinkShape;
        CapsuleShape* mPlayerShape;

        const decimal mTimeStep = decimal(1.0) / decimal(60.0);

        struct Scene {
            PhysicsWorld* world = nullptr;
            RigidBody* ground = nullptr;
            RigidBody* player = nullptr;
            std::vector<RigidBody*> props;
            std::vector<Joint*> joints;
        };

        // ---------- Scene building ---------- //

        Scene makeScene() {
            Scene scene;
            scene.world = mPhysicsCommon.createPhysicsWorld();
            scene.ground = scene.world->createRigidBody(Transform(Vector3(0, decimal(-0.5), 0), Quaternion::identity()));
            scene.ground->setType(BodyType::STATIC);
            scene.ground->addCollider(mGroundShape, Transform::identity());
            scene.player = scene.world->createRigidBody(Transform(Vector3(0, decimal(0.9), 0), Quaternion::identity()));
            scene.player->addCollider(mPlayerShape, Transform::identity());
            scene.player->setAngularLockAxisFactor(Vector3(0, 1, 0));
            return scene;
        }

        RigidBody* addDynamic(Scene& scene, const Vector3& position, CollisionShape* shape) {
            RigidBody* body = scene.world->createRigidBody(Transform(position, Quaternion::identity()));
            body->addCollider(shape, Transform::identity());
            scene.props.push_back(body);
            return body;
        }

        // The props a restart destroys and creates again: a stack of crates, one of them leaning on
        // the player, a hinged pair and a short chain hanging from a static post.
        void addProps(Scene& scene) {
            addDynamic(scene, Vector3(decimal(0.72), decimal(0.42), 0), mCrateShape);
            addDynamic(scene, Vector3(2, decimal(0.42), 0), mCrateShape);
            addDynamic(scene, Vector3(2, decimal(1.26), decimal(0.1)), mCrateShape);

            RigidBody* base = addDynamic(scene, Vector3(-3, decimal(0.42), -2), mCrateShape);
            RigidBody* top = addDynamic(scene, Vector3(-3, decimal(1.26), -2), mCrateShape);
            HingeJointInfo hinge(base, top, Vector3(-3, decimal(0.84), -2), Vector3(1, 0, 0));
            hinge.isLimitEnabled = true;
            hinge.minAngleLimit = decimal(-0.6);
            hinge.maxAngleLimit = decimal(0.6);
            scene.joints.push_back(scene.world->createJoint(hinge));

            RigidBody* post = addDynamic(scene, Vector3(4, 3, -3), mLinkShape);
            post->setType(BodyType::STATIC);
            RigidBody* previous = post;
            for (int k = 1; k <= 3; k++) {
                RigidBody* link = addDynamic(scene, Vector3(4 + decimal(0.18) * k, 3, -3), mLinkShape);
                BallAndSocketJointInfo socket(previous, link, Vector3(4 + decimal(0.18) * k - decimal(0.09), 3, -3));
                scene.joints.push_back(scene.world->createJoint(socket));
                previous = link;
            }
        }

        void destroyProps(Scene& scene) {
            for (Joint* joint : scene.joints) scene.world->destroyJoint(joint);
            scene.joints.clear();
            for (RigidBody* body : scene.props) scene.world->destroyRigidBody(body);
            scene.props.clear();
        }

        // The app's restart: props rebuilt in place, the player put back, optionally the rebuild
        void restart(Scene& scene, bool rebuild) {
            destroyProps(scene);
            addProps(scene);
            scene.player->setTransform(Transform(Vector3(0, decimal(0.9), 0), Quaternion::identity()));
            scene.player->setLinearVelocity(Vector3::zero());
            scene.player->setAngularVelocity(Vector3::zero());
            if (rebuild) scene.world->rebuildInternalState();
        }

        // Play between two replays: darts thrown, the player shoved - a history for the world to keep
        void freePlay(Scene& scene, int ticks) {
            std::vector<RigidBody*> darts;
            for (int tick = 0; tick < ticks; tick++) {
                if (tick % 15 == 3) {
                    RigidBody* dart = scene.world->createRigidBody(Transform(Vector3(decimal(-2 + (tick % 7)), 1, 6), Quaternion::identity()));
                    dart->addCollider(mDartShape, Transform::identity());
                    dart->setLinearVelocity(Vector3(0, 1, -20));
                    darts.push_back(dart);
                }
                if (tick == 10) scene.player->setLinearVelocity(Vector3(-2, 0, 1));
                scene.world->update(mTimeStep);
            }
            for (RigidBody* dart : darts) scene.world->destroyRigidBody(dart);
        }

        // A replay: the same scripted input every time, the physics hash recorded after each tick
        std::vector<PhysicsWorld::StateHash> replay(Scene& scene, int ticks) {
            std::vector<PhysicsWorld::StateHash> trace;
            for (int tick = 0; tick < ticks; tick++) {
                if (tick >= 5 && tick < 40) scene.player->setLinearVelocity(Vector3(decimal(1.5), scene.player->getLinearVelocity().y, 0));
                if (tick == 30) scene.props[1]->applyWorldForceAtCenterOfMass(Vector3(0, 0, 4000));
                if (tick == 50) scene.props[4]->applyWorldForceAtCenterOfMass(Vector3(0, 0, -3000));
                scene.world->update(mTimeStep);
                trace.push_back(scene.world->computeStateHash());
            }
            return trace;
        }

        static bool sameTotals(const std::vector<PhysicsWorld::StateHash>& a, const std::vector<PhysicsWorld::StateHash>& b) {
            if (a.size() != b.size()) return false;
            for (size_t t = 0; t < a.size(); t++) {
                if (a[t].total != b[t].total) return false;
            }
            return true;
        }

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestWorldDeterminism(const std::string& name) : Test(name) {
            mGroundShape = mPhysicsCommon.createBoxShape(Vector3(20, decimal(0.5), 20));
            mCrateShape = mPhysicsCommon.createBoxShape(Vector3(decimal(0.4), decimal(0.4), decimal(0.4)));
            mDartShape = mPhysicsCommon.createBoxShape(Vector3(decimal(0.02), decimal(0.02), decimal(0.35)));
            mLinkShape = mPhysicsCommon.createSphereShape(decimal(0.08));
            mPlayerShape = mPhysicsCommon.createCapsuleShape(decimal(0.3), decimal(1.1));
        }

        /// Run the tests
        void run() {
            testFreshWorldsHashAlike();
            testHashSeesBroadPhaseHistory();
            testRestartMatchesFreshStart();
            testRebuildKeepsObservableState();
            testRebuildResetsJointWarmStart();
            testRebuildResetsSleepTimer();
        }

        /// Two worlds built the same way hash the same on every tick. They share one PhysicsCommon,
        /// so their memory is at different addresses: the hash must not see pointers or entity ids.
        void testFreshWorldsHashAlike() {

            Scene a = makeScene(); addProps(a);
            Scene b = makeScene(); addProps(b);
            rp3d_test(sameTotals(replay(a, 90), replay(b, 90)));

            mPhysicsCommon.destroyPhysicsWorld(a.world);
            mPhysicsCommon.destroyPhysicsWorld(b.world);
        }

        /// The same bodies in the same state, but one world has had a body come and go: the bodies
        /// hash the same, the broad-phase does not - and after a rebuild of both, nothing differs.
        void testHashSeesBroadPhaseHistory() {

            Scene a = makeScene(); addProps(a);

            Scene b = makeScene();
            RigidBody* passing = b.world->createRigidBody(Transform(Vector3(9, 1, 9), Quaternion::identity()));
            passing->addCollider(mCrateShape, Transform::identity());
            addProps(b);
            b.world->destroyRigidBody(passing);

            const PhysicsWorld::StateHash ha = a.world->computeStateHash();
            const PhysicsWorld::StateHash hb = b.world->computeStateHash();
            rp3d_test(ha.bodies == hb.bodies);
            rp3d_test(ha.broadphase != hb.broadphase);
            rp3d_test(ha.total != hb.total);

            a.world->rebuildInternalState();
            b.world->rebuildInternalState();
            rp3d_test(a.world->computeStateHash().total == b.world->computeStateHash().total);
            rp3d_test(sameTotals(replay(a, 90), replay(b, 90)));

            mPhysicsCommon.destroyPhysicsWorld(a.world);
            mPhysicsCommon.destroyPhysicsWorld(b.world);
        }

        /// The case this exists for. A restart in a world with a history, then a replay, matches
        /// the same replay after the same restart in a fresh world - with the rebuild. Without it
        /// the scenario must diverge, or the test would not be testing anything.
        void testRestartMatchesFreshStart() {

            for (int rebuild = 0; rebuild < 2; rebuild++) {

                Scene fresh = makeScene(); addProps(fresh);
                restart(fresh, rebuild == 1);
                const std::vector<PhysicsWorld::StateHash> expected = replay(fresh, 120);

                Scene warm = makeScene(); addProps(warm);
                restart(warm, rebuild == 1);
                replay(warm, 120);
                freePlay(warm, 150);
                restart(warm, rebuild == 1);
                const std::vector<PhysicsWorld::StateHash> actual = replay(warm, 120);

                if (rebuild == 1) {
                    rp3d_test(sameTotals(expected, actual));
                }
                else {
                    rp3d_test(!sameTotals(expected, actual));
                }

                mPhysicsCommon.destroyPhysicsWorld(fresh.world);
                mPhysicsCommon.destroyPhysicsWorld(warm.world);
            }
        }

        /// Everything the public API can see is the same after a rebuild as before it
        void testRebuildKeepsObservableState() {

            Scene scene = makeScene(); addProps(scene);
            scene.props[1]->getCollider(0)->setCollisionCategoryBits(0x0004);
            scene.props[2]->getCollider(0)->getMaterial().setBounciness(decimal(0.7));
            replay(scene, 45);
            scene.props[3]->setIsSleeping(true);

            struct Snapshot { Transform transform; Vector3 linear, angular; bool sleeping; };
            std::vector<Snapshot> before;
            std::vector<RigidBody*> bodies = scene.props;
            bodies.push_back(scene.player);
            for (RigidBody* body : bodies) {
                before.push_back({body->getTransform(), body->getLinearVelocity(), body->getAngularVelocity(), body->isSleeping()});
            }

            scene.world->rebuildInternalState();

            bool same = true;
            for (size_t i = 0; i < bodies.size(); i++) {
                same = same && bodies[i]->getTransform() == before[i].transform;
                same = same && bodies[i]->getLinearVelocity() == before[i].linear;
                same = same && bodies[i]->getAngularVelocity() == before[i].angular;
                same = same && bodies[i]->isSleeping() == before[i].sleeping;
            }
            rp3d_test(same);
            rp3d_test(scene.props[1]->getCollider(0)->getCollisionCategoryBits() == 0x0004);
            rp3d_test(scene.props[2]->getCollider(0)->getMaterial().getBounciness() == decimal(0.7));
            rp3d_test(scene.world->getNbRigidBodies() == bodies.size() + 1);
            rp3d_test(scene.props[0]->getCollider(0)->getBroadPhaseId() != -1);

            // and it still simulates
            replay(scene, 30);
            rp3d_test(scene.player->getTransform().getPosition().y > decimal(0.5));

            mPhysicsCommon.destroyPhysicsWorld(scene.world);
        }

        /// A joint that survives a rebuild starts again from zero warm-start impulse, as a new one would
        void testRebuildResetsJointWarmStart() {

            Scene scene = makeScene(); addProps(scene);
            replay(scene, 30);
            Joint* hanging = scene.joints.back();   // the last chain link holds the others up
            rp3d_test(hanging->getReactionForce(mTimeStep).lengthSquare() > decimal(0.0));

            scene.world->rebuildInternalState();
            rp3d_test(hanging->getReactionForce(mTimeStep).lengthSquare() == decimal(0.0));

            mPhysicsCommon.destroyPhysicsWorld(scene.world);
        }

        /// A resting body keeps counting towards sleep, and nothing public resets the count while it
        /// is awake. After a rebuild it falls asleep as late as a new body would, not early.
        void testRebuildResetsSleepTimer() {

            auto ticksToSleep = [&](Scene& scene, RigidBody* body) {
                for (int tick = 0; tick < 600; tick++) {
                    scene.world->update(mTimeStep);
                    if (body->isSleeping()) return tick;
                }
                return -1;
            };
            // 1 mm into the ground: it settles on the first tick. Placed exactly touching, it
            // bounces a little when put back, and that resets the timer and hides the leak.
            auto restingCrate = [&](Scene& scene) {
                RigidBody* crate = scene.world->createRigidBody(Transform(Vector3(6, decimal(0.399), 6), Quaternion::identity()));
                crate->addCollider(mCrateShape, Transform::identity());
                return crate;
            };
            auto putBack = [&](RigidBody* crate) {
                crate->setTransform(Transform(Vector3(6, decimal(0.399), 6), Quaternion::identity()));
                crate->setLinearVelocity(Vector3::zero());
                crate->setAngularVelocity(Vector3::zero());
            };

            Scene fresh = makeScene();
            RigidBody* freshCrate = restingCrate(fresh);
            fresh.world->rebuildInternalState();
            const int expected = ticksToSleep(fresh, freshCrate);
            rp3d_test(expected > 30);

            for (int rebuild = 0; rebuild < 2; rebuild++) {
                Scene warm = makeScene();
                RigidBody* crate = restingCrate(warm);
                for (int tick = 0; tick < 30; tick++) warm.world->update(mTimeStep);   // resting, counting
                rp3d_test(!crate->isSleeping());
                putBack(crate);
                if (rebuild == 1) {
                    warm.world->rebuildInternalState();
                    rp3d_test(ticksToSleep(warm, crate) == expected);
                }
                else {
                    rp3d_test(ticksToSleep(warm, crate) < expected);
                }
                mPhysicsCommon.destroyPhysicsWorld(warm.world);
            }

            mPhysicsCommon.destroyPhysicsWorld(fresh.world);
        }
};

}

#endif
