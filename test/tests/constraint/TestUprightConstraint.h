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

#ifndef TEST_UPRIGHT_CONSTRAINT_H
#define TEST_UPRIGHT_CONSTRAINT_H

// Libraries
#include "Test.h"
#include <reactphysics3d/reactphysics3d.h>
#include <vector>
#include <cmath>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestUprightConstraint
/**
 * Unit test for the AngularAxisConstraintPart and the UprightConstraint: a body floating in a
 * world without gravity (like an arcade spaceship) that is kept upright, as a hard cone or as a
 * self-righting spring.
 */
class TestUprightConstraint : public Test {

    private :

        // ---------- Attributes ---------- //

        PhysicsCommon mPhysicsCommon;
        PhysicsWorld* mWorld;
        RigidBody* mBody;
        UprightConstraint* mConstraint;

        static constexpr decimal TIME_STEP = decimal(1.0) / decimal(60.0);
        static constexpr decimal DEG = PI_RP3D / decimal(180.0);

        // ---------- Methods ---------- //

        /// A 1 kg body with a diagonal inertia tensor, rolled about z by rollAngle, in zero gravity
        void createScene(decimal rollAngle, const UprightConstraintSettings& settings) {

            mWorld = mPhysicsCommon.createPhysicsWorld();
            mWorld->setIsGravityEnabled(false);

            mBody = mWorld->createRigidBody(Transform(Vector3(0, 0, 0), Quaternion::fromEulerAngles(0, 0, rollAngle)));
            mBody->setMass(decimal(1.0));
            mBody->setLocalInertiaTensor(Vector3(0.5, 0.3, 0.6));
            mBody->setIsAllowedToSleep(false);

            mConstraint = mWorld->createUprightConstraint(mBody, settings);
        }

        void destroyScene() {
            if (mConstraint != nullptr) mWorld->destroyUprightConstraint(mConstraint);
            mWorld->destroyRigidBody(mBody);
            mPhysicsCommon.destroyPhysicsWorld(mWorld);
            mWorld = nullptr;
            mConstraint = nullptr;
        }

        void step(int nbSteps) {
            for (int i = 0; i < nbSteps; i++) {
                mWorld->update(TIME_STEP);
            }
        }

        /// Angle (rad) between the body up axis and world up
        decimal tilt() const {
            const Vector3 up = mBody->getTransform().getOrientation() * Vector3(0, 1, 0);
            return std::acos(clamp(up.y, decimal(-1.0), decimal(1.0)));
        }

        /// Signed roll (rad) of the body up axis in the xy plane, positive towards -x (a positive rotation about z)
        decimal signedRoll() const {
            const Vector3 up = mBody->getTransform().getOrientation() * Vector3(0, 1, 0);
            return std::atan2(-up.x, up.y);
        }

        /// Average period of a signal sampled every TIME_STEP, from its zero crossings
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
        TestUprightConstraint(const std::string& name) : Test(name), mWorld(nullptr), mBody(nullptr), mConstraint(nullptr) {

        }

        /// Run the tests
        void run() {
            testAngularPart();
            testCreation();
            testHardConeRightsABodyStartedOutside();
            testHardConeStopsATiltingBody();
            testHardConeLeavesYawFree();
            testSoftReturnsUpright();
            testSoftInsideConeIsFree();
            testDestroyBodyDestroysConstraint();
        }

        /// The angular part alone: it zeroes the relative angular velocity of two bodies about an
        /// axis in one solve and conserves angular momentum about that axis
        void testAngularPart() {
            const Vector3 axis(0, 0, 1);
            Vector3 w1(0, 0, 3), w2(0, 0, -1);
            Vector3 v(0, 0, 0);
            const Matrix3x3 invI1(2, 0, 0, 0, 2, 0, 0, 0, 2);     // I1 = 0.5
            const Matrix3x3 invI2(0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5); // I2 = 2
            AxisConstraintBody body1(&v, &w1, decimal(1.0), &invI1);
            AxisConstraintBody body2(&v, &w2, decimal(1.0), &invI2);

            const decimal momentumBefore = decimal(0.5) * w1.z + decimal(2.0) * w2.z;

            AngularAxisConstraintPart part;
            part.computeConstraintProperties(body1, body2, axis);
            rp3d_test(part.isActive());
            rp3d_test(approxEqual(part.getEffectiveMass(), decimal(1.0) / decimal(2.5), decimal(1e-5)));
            rp3d_test(part.solveVelocityConstraint(body1, body2, DECIMAL_SMALLEST, DECIMAL_LARGEST));
            rp3d_test(approxEqual(w1.z, w2.z, decimal(1e-5)));
            rp3d_test(approxEqual(decimal(0.5) * w1.z + decimal(2.0) * w2.z, momentumBefore, decimal(1e-5)));
            rp3d_test(!part.solveVelocityConstraint(body1, body2, DECIMAL_SMALLEST, DECIMAL_LARGEST));

            // Against the world (zero inertia) a one-sided constraint stops a negative spin and lets a positive one go
            const Matrix3x3 zero = Matrix3x3::zero();
            Vector3 wWorld(0, 0, 0);
            AxisConstraintBody world(&v, &wWorld, decimal(0.0), &zero);
            Vector3 w(0, 0, -2);
            AxisConstraintBody body(&v, &w, decimal(1.0), &invI2);
            part.computeConstraintProperties(world, body, axis);
            part.resetTotalLambda();
            part.solveVelocityConstraint(world, body, decimal(0.0), DECIMAL_LARGEST);
            rp3d_test(approxEqual(w.z, decimal(0.0), decimal(1e-5)));
            rp3d_test(approxEqual(part.getTotalLambda(), decimal(4.0), decimal(1e-5)));
            rp3d_test(Vector3::approxEqual(wWorld, Vector3(0, 0, 0)));
            w.setAllValues(0, 0, 2);
            part.resetTotalLambda();
            rp3d_test(!part.solveVelocityConstraint(world, body, decimal(0.0), DECIMAL_LARGEST));
            rp3d_test(approxEqual(w.z, decimal(2.0)));
        }

        /// The constraint is registered in the world with its settings
        void testCreation() {
            UprightConstraintSettings settings;
            settings.maxAngle = 30 * DEG;
            createScene(decimal(0.0), settings);

            rp3d_test(mConstraint != nullptr);
            rp3d_test(mWorld->getNbUprightConstraints() == 1);
            rp3d_test(mWorld->getUprightConstraint(0) == mConstraint);
            rp3d_test(mConstraint->getBody() == mBody);
            rp3d_test(approxEqual(mConstraint->getMaxAngle(), 30 * DEG));
            rp3d_test(Vector3::approxEqual(mConstraint->getLocalAxis(), Vector3(0, 1, 0)));
            rp3d_test(Vector3::approxEqual(mConstraint->getWorldAxis(), Vector3(0, 1, 0)));
            rp3d_test(!mConstraint->getSpringSettings().isSoft());
            rp3d_test(!mConstraint->isActive());

            // Upright: stepping does nothing
            step(10);
            rp3d_test(!mConstraint->isActive());
            rp3d_test(tilt() < decimal(1e-4));
            rp3d_test(mBody->getAngularVelocity().length() < decimal(1e-6));

            destroyScene();
        }

        /// A body created leaning 50 degrees with a 30 degree cone is pushed back to the edge of
        /// the cone (position correction) and stays there
        void testHardConeRightsABodyStartedOutside() {
            UprightConstraintSettings settings;
            settings.maxAngle = 30 * DEG;
            createScene(50 * DEG, settings);
            rp3d_test(approxEqual(tilt(), 50 * DEG, decimal(1e-4)));

            step(60);
            rp3d_test(tilt() < 30 * DEG + decimal(0.5) * DEG);
            rp3d_test(tilt() > 30 * DEG - decimal(2.0) * DEG);
            rp3d_test(std::abs(mConstraint->getCurrentAngle() - tilt()) < decimal(0.5) * DEG);
            // Only rolled back, not yawed
            rp3d_test(std::abs(mBody->getAngularVelocity().y) < decimal(1e-3));

            destroyScene();
        }

        /// A body spinning towards the cone edge is stopped there: the tilt never exceeds the limit
        /// and the tilting velocity is taken away
        void testHardConeStopsATiltingBody() {
            UprightConstraintSettings settings;
            settings.maxAngle = 30 * DEG;
            createScene(decimal(0.0), settings);
            mBody->setAngularVelocity(Vector3(0, 0, 2));

            decimal maxTilt = 0;
            for (int i = 0; i < 120; i++) {
                mWorld->update(TIME_STEP);
                maxTilt = std::max(maxTilt, tilt());
            }
            rp3d_test(maxTilt < 30 * DEG + decimal(0.5) * DEG);
            rp3d_test(tilt() > 29 * DEG);
            rp3d_test(mBody->getAngularVelocity().length() < decimal(0.05));
            rp3d_test(mConstraint->isActive());
            // The righting torque acted about +z... no: about the axis that reduces the tilt. Body
            // rolled about +z, so its up went towards -x, and (up x worldUp) points along -z.
            rp3d_test(mConstraint->getReactionTorque(TIME_STEP).z <= decimal(0.0));

            destroyScene();
        }

        /// Yaw is not constrained: a body spinning about the world axis keeps spinning
        void testHardConeLeavesYawFree() {
            UprightConstraintSettings settings;
            settings.maxAngle = 30 * DEG;
            createScene(20 * DEG, settings);
            mBody->setAngularVelocity(Vector3(0, 3, 0));
            step(120);

            rp3d_test(std::abs(mBody->getAngularVelocity().y - decimal(3.0)) < decimal(0.05));
            rp3d_test(tilt() < 30 * DEG + decimal(0.5) * DEG);
            rp3d_test(!mConstraint->isActive());

            destroyScene();
        }

        /// A soft constraint with a zero cone springs the body back to upright at the configured
        /// frequency, damping the swing, and ends up upright
        void testSoftReturnsUpright() {
            UprightConstraintSettings settings;
            settings.maxAngle = decimal(0.0);
            settings.spring = SpringSettings::fromFrequencyAndDampingRatio(1.0, 0.1);
            createScene(40 * DEG, settings);

            std::vector<decimal> roll;
            for (int i = 0; i < 300; i++) {
                mWorld->update(TIME_STEP);
                roll.push_back(signedRoll());
            }
            const decimal period = measurePeriod(roll);
            rp3d_test(std::abs(period - decimal(1.0)) < decimal(0.05));
            // Decaying: the last swing is much smaller than the first
            rp3d_test(std::abs(roll.back()) < 10 * DEG);

            // Critically damped: no overshoot and upright within a couple of seconds
            destroyScene();
            settings.spring = SpringSettings::fromFrequencyAndDampingRatio(1.0, 1.0);
            createScene(40 * DEG, settings);
            bool overshoot = false;
            for (int i = 0; i < 180; i++) {
                mWorld->update(TIME_STEP);
                if (signedRoll() < -decimal(0.5) * DEG) overshoot = true;
            }
            rp3d_test(!overshoot);
            rp3d_test(tilt() < decimal(0.5) * DEG);

            destroyScene();
        }

        /// A soft constraint with a cone leaves the body alone inside the cone
        void testSoftInsideConeIsFree() {
            UprightConstraintSettings settings;
            settings.maxAngle = 20 * DEG;
            settings.spring = SpringSettings::fromFrequencyAndDampingRatio(1.0, 0.5);
            createScene(10 * DEG, settings);
            step(60);

            rp3d_test(approxEqual(tilt(), 10 * DEG, decimal(1e-3)));
            rp3d_test(!mConstraint->isActive());
            rp3d_test(mBody->getAngularVelocity().length() < decimal(1e-6));

            // Pushed outside it is brought back to the edge
            mBody->setTransform(Transform(Vector3(0, 0, 0), Quaternion::fromEulerAngles(0, 0, 40 * DEG)));
            step(240);
            rp3d_test(tilt() < 20 * DEG + decimal(1.0) * DEG);

            destroyScene();
        }

        /// Destroying the body takes the constraint with it
        void testDestroyBodyDestroysConstraint() {
            UprightConstraintSettings settings;
            createScene(decimal(0.0), settings);
            step(5);

            mWorld->destroyRigidBody(mBody);
            rp3d_test(mWorld->getNbUprightConstraints() == 0);
            mConstraint = nullptr;
            step(5);

            mPhysicsCommon.destroyPhysicsWorld(mWorld);
            mWorld = nullptr;
        }
};

}

#endif
