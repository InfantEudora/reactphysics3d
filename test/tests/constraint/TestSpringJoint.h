/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2026 Daniel Chappuis                                       *
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

#ifndef TEST_SPRING_JOINT_H
#define TEST_SPRING_JOINT_H

// Libraries
#include "Test.h"
#include <reactphysics3d/reactphysics3d.h>
#include <vector>
#include <cmath>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestSpringJoint
/**
 * Unit test for the SpringJoint class, run through a real PhysicsWorld: a body hanging from a
 * static anchor on a spring (or a rigid rod), checked against the analytical solution.
 */
class TestSpringJoint : public Test {

    private :

        // ---------- Attributes ---------- //

        PhysicsCommon mPhysicsCommon;
        PhysicsWorld* mWorld;
        RigidBody* mAnchor;
        RigidBody* mBody;
        SpringJoint* mJoint;

        static constexpr decimal TIME_STEP = decimal(1.0) / decimal(60.0);
        static constexpr decimal MASS = decimal(2.0);
        static constexpr decimal GRAVITY = decimal(9.81);

        // ---------- Methods ---------- //

        /// Create a world with a static anchor at (0, 5, 0) and a 2 kg body at bodyPosition,
        /// joined by a spring joint between the anchor and the body center
        void createScene(const Vector3& bodyPosition, const SpringSettings& spring, decimal restLength = decimal(-1.0)) {

            mWorld = mPhysicsCommon.createPhysicsWorld();
            mWorld->setGravity(Vector3(0, -GRAVITY, 0));

            mAnchor = mWorld->createRigidBody(Transform(Vector3(0, 5, 0), Quaternion::identity()));
            mAnchor->setType(BodyType::STATIC);

            mBody = mWorld->createRigidBody(Transform(bodyPosition, Quaternion::identity()));
            mBody->setMass(MASS);
            mBody->setLocalInertiaTensor(Vector3(0.1, 0.1, 0.1));
            mBody->setIsAllowedToSleep(false);

            SpringJointInfo info(mAnchor, mBody, Vector3(0, 5, 0), bodyPosition, spring, restLength);
            mJoint = dynamic_cast<SpringJoint*>(mWorld->createJoint(info));
        }

        void destroyScene() {
            mWorld->destroyJoint(mJoint);
            mWorld->destroyRigidBody(mBody);
            mWorld->destroyRigidBody(mAnchor);
            mPhysicsCommon.destroyPhysicsWorld(mWorld);
            mWorld = nullptr;
        }

        /// Average period of the oscillation of a signal sampled every TIME_STEP, from its zero crossings
        static decimal measurePeriod(const std::vector<decimal>& signal) {
            std::vector<decimal> crossings;
            for (size_t i = 1; i < signal.size(); i++) {
                if ((signal[i-1] < 0) != (signal[i] < 0)) {
                    crossings.push_back((decimal(i) - signal[i] / (signal[i] - signal[i-1])) * TIME_STEP);
                }
            }
            if (crossings.size() < 3) return decimal(0.0);
            decimal sum = 0;
            int count = 0;
            for (size_t i = 2; i < crossings.size(); i++) {
                sum += crossings[i] - crossings[i-2];
                count++;
            }
            return sum / decimal(count);
        }

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestSpringJoint(const std::string& name) : Test(name), mWorld(nullptr), mAnchor(nullptr), mBody(nullptr), mJoint(nullptr) {

        }

        /// Run the tests
        void run() {
            testCreation();
            testGravityEquilibrium();
            testFrequency();
            testRigidRod();
            testSetters();
        }

        /// The joint is created with the right type, anchors and rest length
        void testCreation() {
            createScene(Vector3(0, 4, 0), SpringSettings::fromStiffnessAndDamping(500.0, 50.0));

            rp3d_test(mJoint != nullptr);
            rp3d_test(mJoint->getType() == JointType::SPRINGJOINT);
            rp3d_test(mJoint->getBody1() == mAnchor);
            rp3d_test(mJoint->getBody2() == mBody);
            rp3d_test(approxEqual(mJoint->getRestLength(), decimal(1.0), decimal(1e-5)));
            rp3d_test(approxEqual(mJoint->getCurrentLength(), decimal(1.0), decimal(1e-5)));
            rp3d_test(Vector3::approxEqual(mJoint->getLocalAnchorPointBody1(), Vector3(0, 0, 0), decimal(1e-5)));
            rp3d_test(Vector3::approxEqual(mJoint->getLocalAnchorPointBody2(), Vector3(0, 0, 0), decimal(1e-5)));
            rp3d_test(mJoint->getSpringSettings().mode == SpringMode::STIFFNESS_AND_DAMPING);
            rp3d_test(!mJoint->to_string().empty());

            destroyScene();
        }

        /// Hanging under gravity the body settles at rest length + m g / k and the joint reports
        /// a reaction force equal to the weight
        void testGravityEquilibrium() {
            const decimal stiffness = 500.0;
            createScene(Vector3(0, 4, 0), SpringSettings::fromStiffnessAndDamping(stiffness, 50.0));

            for (int i = 0; i < 300; i++) {
                mWorld->update(TIME_STEP);
            }

            const decimal expectedLength = decimal(1.0) + MASS * GRAVITY / stiffness;
            rp3d_test(std::abs(mJoint->getCurrentLength() - expectedLength) < decimal(0.001));
            rp3d_test(std::abs(mBody->getTransform().getPosition().y - (decimal(5.0) - expectedLength)) < decimal(0.001));
            rp3d_test(mBody->getLinearVelocity().length() < decimal(0.001));

            // The force on body 2 must be the weight, pointing up (towards the anchor)
            const Vector3 force = mJoint->getReactionForce(TIME_STEP);
            rp3d_test(std::abs(force.y - MASS * GRAVITY) < MASS * GRAVITY * decimal(0.02));
            rp3d_test(std::abs(force.x) < decimal(0.01) && std::abs(force.z) < decimal(0.01));

            destroyScene();
        }

        /// A 1 Hz spring with the body displaced from its rest length oscillates with a 1 s period
        void testFrequency() {
            createScene(Vector3(0, 3.9, 0), SpringSettings::fromFrequencyAndDampingRatio(1.0, 0.0), decimal(1.0));
            mWorld->setIsGravityEnabled(false);

            std::vector<decimal> error;
            for (int i = 0; i < 300; i++) {
                mWorld->update(TIME_STEP);
                error.push_back(mJoint->getCurrentLength() - decimal(1.0));
            }

            const decimal period = measurePeriod(error);
            rp3d_test(std::abs(period - decimal(1.0)) < decimal(0.03));
            // The body must stay on the vertical line through the anchor
            rp3d_test(std::abs(mBody->getTransform().getPosition().x) < decimal(1e-4));

            destroyScene();
        }

        /// A joint with no spring is a rigid rod: released horizontally it swings as a pendulum
        /// while the anchor distance stays at the rest length
        void testRigidRod() {
            createScene(Vector3(1, 5, 0), SpringSettings());

            decimal maxLengthError = 0;
            decimal minY = decimal(5.0);
            for (int i = 0; i < 180; i++) {
                mWorld->update(TIME_STEP);
                maxLengthError = std::max(maxLengthError, std::abs(mJoint->getCurrentLength() - decimal(1.0)));
                minY = std::min(minY, mBody->getTransform().getPosition().y);
            }

            rp3d_test(maxLengthError < decimal(0.02));
            rp3d_test(minY < decimal(4.2));

            destroyScene();
        }

        /// Changing the rest length and the spring settings takes effect
        void testSetters() {
            const decimal stiffness = 500.0;
            createScene(Vector3(0, 4, 0), SpringSettings::fromStiffnessAndDamping(stiffness, 50.0));

            mJoint->setRestLength(decimal(1.2));
            rp3d_test(approxEqual(mJoint->getRestLength(), decimal(1.2)));
            for (int i = 0; i < 300; i++) {
                mWorld->update(TIME_STEP);
            }
            rp3d_test(std::abs(mJoint->getCurrentLength() - (decimal(1.2) + MASS * GRAVITY / stiffness)) < decimal(0.001));

            // Twice the stiffness: half the sag
            mJoint->setSpringSettings(SpringSettings::fromStiffnessAndDamping(decimal(2.0) * stiffness, 50.0));
            rp3d_test(approxEqual(mJoint->getSpringSettings().frequencyOrStiffness, decimal(2.0) * stiffness));
            for (int i = 0; i < 300; i++) {
                mWorld->update(TIME_STEP);
            }
            rp3d_test(std::abs(mJoint->getCurrentLength() - (decimal(1.2) + MASS * GRAVITY / (decimal(2.0) * stiffness))) < decimal(0.001));

            destroyScene();
        }
};

}

#endif
