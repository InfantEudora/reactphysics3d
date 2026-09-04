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

#ifndef TEST_VEHICLE_CONSTRAINT_H
#define TEST_VEHICLE_CONSTRAINT_H

// Libraries
#include "Test.h"
#include <reactphysics3d/reactphysics3d.h>
#include <cmath>

/// Reactphysics3D namespace
namespace reactphysics3d {

// Class TestVehicleConstraint
/**
 * Unit test for the VehicleConstraint class: a four-wheeled chassis dropped onto a static
 * floor, checked against the analytical equilibrium (suspension carries the weight, car level,
 * wheels resting their tread on the ground), then rolling, sliding sideways and bottoming out.
 */
class TestVehicleConstraint : public Test {

    private :

        // ---------- Attributes ---------- //

        PhysicsCommon mPhysicsCommon;
        PhysicsWorld* mWorld;
        RigidBody* mFloor;
        RigidBody* mChassis;
        Collider* mFloorCollider;
        Collider* mChassisCollider;
        VehicleConstraint* mVehicle;

        static constexpr decimal TIME_STEP = decimal(1.0) / decimal(60.0);
        static constexpr decimal GRAVITY = decimal(9.81);
        static constexpr decimal MASS = decimal(1000.0);
        static constexpr decimal RADIUS = decimal(0.3);
        static constexpr decimal MIN_LENGTH = decimal(0.2);
        static constexpr decimal MAX_LENGTH = decimal(0.5);
        static constexpr decimal WHEEL_INERTIA = decimal(0.9);

        // ---------- Methods ---------- //

        /// Create a floor (top surface through the origin, rotated by floorOrientation) and a
        /// 1000 kg box chassis with four wheels, spawned `height` above the floor along its normal
        void createScene(const Quaternion& floorOrientation, decimal height) {

            mWorld = mPhysicsCommon.createPhysicsWorld();
            mWorld->setGravity(Vector3(0, -GRAVITY, 0));

            const Vector3 floorNormal = floorOrientation * Vector3(0, 1, 0);

            // Floor: 40 x 1 x 40 box whose top face passes through the origin
            mFloor = mWorld->createRigidBody(Transform(floorNormal * decimal(-0.5), floorOrientation));
            mFloor->setType(BodyType::STATIC);
            BoxShape* floorShape = mPhysicsCommon.createBoxShape(Vector3(20, 0.5, 20));
            mFloorCollider = mFloor->addCollider(floorShape, Transform::identity());

            // Chassis: 1.6 x 0.5 x 3.6 box, 1000 kg, forward along +z
            mChassis = mWorld->createRigidBody(Transform(floorNormal * height, floorOrientation));
            BoxShape* chassisShape = mPhysicsCommon.createBoxShape(Vector3(0.8, 0.25, 1.8));
            mChassisCollider = mChassis->addCollider(chassisShape, Transform::identity());
            mChassisCollider->getMaterial().setMassDensity(MASS / (decimal(1.6) * decimal(0.5) * decimal(3.6)));
            mChassis->updateMassPropertiesFromColliders();
            mChassis->setIsAllowedToSleep(false);

            // Four wheels at the corners, struts hanging straight down from y = -0.2
            mVehicle = mWorld->createVehicle(mChassis);
            for (int i = 0; i < 4; i++) {
                VehicleWheelSettings wheel;
                wheel.position = Vector3((i % 2 == 0) ? -0.8 : 0.8, -0.2, (i < 2) ? 1.4 : -1.4);
                wheel.suspensionDirection = Vector3(0, -1, 0);
                wheel.suspensionMinLength = MIN_LENGTH;
                wheel.suspensionMaxLength = MAX_LENGTH;
                wheel.radius = RADIUS;
                wheel.inertia = WHEEL_INERTIA;
                wheel.suspensionSpring = SpringSettings::fromFrequencyAndDampingRatio(1.5, 0.5);
                mVehicle->addWheel(wheel);
            }
        }

        void destroyScene() {
            if (mVehicle != nullptr) mWorld->destroyVehicle(mVehicle);
            mWorld->destroyRigidBody(mChassis);
            mWorld->destroyRigidBody(mFloor);
            mPhysicsCommon.destroyPhysicsWorld(mWorld);
            mWorld = nullptr;
            mVehicle = nullptr;
        }

        void step(int nbSteps) {
            for (int i = 0; i < nbSteps; i++) {
                mWorld->update(TIME_STEP);
            }
        }

        /// Forward direction of the chassis in world space
        Vector3 forward() const {
            return mChassis->getTransform().getOrientation() * Vector3(0, 0, 1);
        }

        /// Up direction of the chassis in world space
        Vector3 up() const {
            return mChassis->getTransform().getOrientation() * Vector3(0, 1, 0);
        }

        /// Right direction of the chassis in world space
        Vector3 right() const {
            return mChassis->getTransform().getOrientation() * Vector3(1, 0, 0);
        }

    public :

        // ---------- Methods ---------- //

        /// Constructor
        TestVehicleConstraint(const std::string& name) : Test(name), mWorld(nullptr), mFloor(nullptr), mChassis(nullptr),
                                                           mFloorCollider(nullptr), mChassisCollider(nullptr), mVehicle(nullptr) {

        }

        /// Run the tests
        void run() {
            testCreation();
            testRestOnFlatGround();
            testInTheAir();
            testFreeWheelSpinDown();
            testRollsDownSlope();
            testRollingAndLateralGrip();
            testHardStop();
            testDestroyBodyDestroysVehicle();
        }

        /// The vehicle is registered in the world and its wheels start at full droop, not spinning
        void testCreation() {
            createScene(Quaternion::identity(), decimal(1.3));

            rp3d_test(mVehicle != nullptr);
            rp3d_test(mWorld->getNbVehicles() == 1);
            rp3d_test(mWorld->getVehicle(0) == mVehicle);
            rp3d_test(mVehicle->getBody() == mChassis);
            rp3d_test(mVehicle->getNbWheels() == 4);
            for (uint32 i = 0; i < 4; i++) {
                const VehicleWheel& wheel = mVehicle->getWheel(i);
                rp3d_test(!wheel.hasContact());
                rp3d_test(!wheel.hasHitHardStop());
                rp3d_test(approxEqual(wheel.getSuspensionLength(), MAX_LENGTH));
                rp3d_test(approxEqual(wheel.getNormalImpulse(), decimal(0.0)));
                rp3d_test(approxEqual(wheel.getAngularVelocity(), decimal(0.0)));
            }
            rp3d_test(Vector3::approxEqual(mVehicle->getLocalUp(), Vector3(0, 1, 0)));
            rp3d_test(Vector3::approxEqual(mVehicle->getLocalForward(), Vector3(0, 0, 1)));

            destroyScene();
        }

        /// Dropped onto flat ground the car settles level, with all four wheels touching, the
        /// suspension carrying exactly the weight, every tread resting on the floor and no tire
        /// force or wheel spin
        void testRestOnFlatGround() {
            createScene(Quaternion::identity(), decimal(1.3));
            step(360);

            const decimal weight = MASS * GRAVITY;
            rp3d_test(std::abs(mVehicle->getTotalSuspensionForce(TIME_STEP) - weight) < weight * decimal(0.02));

            decimal minLength = MAX_LENGTH, maxLength = 0;
            for (uint32 i = 0; i < 4; i++) {
                const VehicleWheel& wheel = mVehicle->getWheel(i);
                rp3d_test(wheel.hasContact());
                rp3d_test(!wheel.hasHitHardStop());
                rp3d_test(wheel.getContactBody() == mFloor);
                rp3d_test(Vector3::approxEqual(wheel.getContactNormal(), Vector3(0, 1, 0), decimal(1e-3)));
                rp3d_test(Vector3::approxEqual(wheel.getContactLongitudinal(), Vector3(0, 0, 1), decimal(1e-3)));
                rp3d_test(Vector3::approxEqual(wheel.getContactLateral(), Vector3(1, 0, 0), decimal(1e-3)));
                rp3d_test(std::abs(wheel.getContactPoint().y) < decimal(1e-3));
                // Each wheel carries a quarter of the weight, with no tire force at rest
                rp3d_test(std::abs(wheel.getSuspensionImpulse() / TIME_STEP - weight / decimal(4.0)) < weight * decimal(0.02));
                rp3d_test(std::abs(wheel.getLongitudinalImpulse() / TIME_STEP) < decimal(1.0));
                rp3d_test(std::abs(wheel.getLateralImpulse() / TIME_STEP) < decimal(1.0));
                rp3d_test(std::abs(wheel.getAngularVelocity()) < decimal(0.01));
                // The tread rests on the floor: wheel centre one radius above it
                rp3d_test(std::abs(mVehicle->getWheelCenterWorld(i).y - RADIUS) < decimal(0.005));
                minLength = std::min(minLength, wheel.getSuspensionLength());
                maxLength = std::max(maxLength, wheel.getSuspensionLength());
            }
            // Compressed, within travel, and the same on all wheels
            rp3d_test(maxLength < MAX_LENGTH - decimal(0.01));
            rp3d_test(minLength > MIN_LENGTH);
            rp3d_test(maxLength - minLength < decimal(0.002));

            // Level and at rest
            rp3d_test(up().y > decimal(0.9999));
            rp3d_test(mChassis->getLinearVelocity().length() < decimal(0.01));
            rp3d_test(mChassis->getAngularVelocity().length() < decimal(0.01));

            destroyScene();
        }

        /// A car far above the ground is in free fall: no contacts, no forces
        void testInTheAir() {
            createScene(Quaternion::identity(), decimal(5.0));
            step(10);

            for (uint32 i = 0; i < 4; i++) {
                rp3d_test(!mVehicle->getWheel(i).hasContact());
                rp3d_test(approxEqual(mVehicle->getWheel(i).getSuspensionLength(), MAX_LENGTH));
            }
            rp3d_test(approxEqual(mVehicle->getTotalSuspensionForce(TIME_STEP), decimal(0.0)));
            // Free fall: v = g t after 10 steps
            rp3d_test(std::abs(mChassis->getLinearVelocity().y + GRAVITY * TIME_STEP * decimal(10.0)) < decimal(0.01));

            destroyScene();
        }

        /// A spinning wheel in the air slows down by its angular damping only: w *= (1 - c dt) each step
        void testFreeWheelSpinDown() {
            // High enough not to reach the ground within the test (1 s of free fall is 4.9 m)
            createScene(Quaternion::identity(), decimal(30.0));
            mVehicle->getWheel(0).setAngularVelocity(decimal(10.0));
            const decimal damping = mVehicle->getWheel(0).getSettings().angularDamping;
            step(60);

            const decimal expected = decimal(10.0) * std::pow(decimal(1.0) - damping * TIME_STEP, decimal(60.0));
            rp3d_test(std::abs(mVehicle->getWheel(0).getAngularVelocity() - expected) < decimal(0.01));
            rp3d_test(mVehicle->getWheel(0).getRotationAngle() >= decimal(0.0));
            rp3d_test(mVehicle->getWheel(0).getRotationAngle() < decimal(2.0) * PI_RP3D);
            rp3d_test(approxEqual(mVehicle->getWheel(1).getAngularVelocity(), decimal(0.0)));

            destroyScene();
        }

        /// On a 10 degree slope (facing downhill) a car with free wheels rolls down: the suspension
        /// carries m g cos(theta), the wheels roll without slipping, and the chassis accelerates at
        /// g sin(theta) reduced by the rotational inertia of the wheels
        void testRollsDownSlope() {
            const decimal angle = decimal(10.0) * PI_RP3D / decimal(180.0);
            // Rotation about x: the chassis forward (+z) points downhill along the slope
            createScene(Quaternion::fromEulerAngles(angle, 0, 0), decimal(1.3));
            step(150);

            const decimal expectedNormal = MASS * GRAVITY * std::cos(angle);
            rp3d_test(std::abs(mVehicle->getTotalSuspensionForce(TIME_STEP) - expectedNormal) < expectedNormal * decimal(0.03));

            // Rolling down the slope, still level with it, not drifting sideways
            const decimal forwardSpeed = mChassis->getLinearVelocity().dot(forward());
            rp3d_test(forwardSpeed > decimal(3.0) && forwardSpeed < decimal(4.5));
            rp3d_test(std::abs(mChassis->getLinearVelocity().dot(right())) < decimal(0.02));
            rp3d_test(std::abs(up().dot(mFloor->getTransform().getOrientation() * Vector3(0, 1, 0)) - decimal(1.0)) < decimal(1e-3));

            // Wheels roll without slipping: surface speed equals the ground speed of the chassis
            for (uint32 i = 0; i < 4; i++) {
                const VehicleWheel& wheel = mVehicle->getWheel(i);
                rp3d_test(wheel.hasContact());
                rp3d_test(std::abs(wheel.getAngularVelocity() * RADIUS - forwardSpeed) < forwardSpeed * decimal(0.05));
            }

            // Acceleration: g sin(theta) * m / (m + 4 I / r^2), measured over the next 60 steps
            const decimal speedBefore = forwardSpeed;
            step(60);
            const decimal speedAfter = mChassis->getLinearVelocity().dot(forward());
            const decimal expectedAcceleration = GRAVITY * std::sin(angle) * MASS / (MASS + decimal(4.0) * WHEEL_INERTIA / (RADIUS * RADIUS));
            rp3d_test(std::abs((speedAfter - speedBefore) / decimal(1.0) - expectedAcceleration) < expectedAcceleration * decimal(0.05));

            destroyScene();
        }

        /// Pushed forward the car keeps rolling (only the wheels spinning up takes some speed);
        /// pushed sideways the tires stop it within a fraction of a second
        void testRollingAndLateralGrip() {
            createScene(Quaternion::identity(), decimal(1.3));
            step(180);

            // Forward push: free-rolling wheels, so the momentum is shared with the wheel inertia
            // and the rest is kept: v = v0 * m / (m + 4 I / r^2)
            mChassis->setLinearVelocity(Vector3(0, 0, 5));
            step(90);
            const decimal expectedSpeed = decimal(5.0) * MASS / (MASS + decimal(4.0) * WHEEL_INERTIA / (RADIUS * RADIUS));
            const decimal forwardSpeed = mChassis->getLinearVelocity().z;
            rp3d_test(std::abs(forwardSpeed - expectedSpeed) < expectedSpeed * decimal(0.03));
            rp3d_test(std::abs(mChassis->getLinearVelocity().x) < decimal(0.02));
            for (uint32 i = 0; i < 4; i++) {
                rp3d_test(std::abs(mVehicle->getWheel(i).getAngularVelocity() * RADIUS - forwardSpeed) < forwardSpeed * decimal(0.03));
            }

            // Sideways push: the tires stop the contact patches almost at once, but the impulse acts
            // at ground level, well below the centre of mass, so the body rolls on its suspension
            // and sways for a couple of cycles (1.5 Hz, ratio 0.5) before the sideways motion is gone
            mChassis->setLinearVelocity(Vector3(2, 0, 0));
            step(120);
            rp3d_test(std::abs(mChassis->getLinearVelocity().x) < decimal(0.05));
            rp3d_test(up().y > decimal(0.98));
            // The wheels were still spinning at the forward speed when the push replaced the
            // velocity, so they hand a little of that back: v = v_wheels * (4 I / r^2) / (m + 4 I / r^2)
            const decimal expectedDrift = forwardSpeed * (decimal(4.0) * WHEEL_INERTIA / (RADIUS * RADIUS)) / (MASS + decimal(4.0) * WHEEL_INERTIA / (RADIUS * RADIUS));
            rp3d_test(std::abs(mChassis->getLinearVelocity().z - expectedDrift) < decimal(0.05));

            destroyScene();
        }

        /// Dropped from high up the suspension bottoms out: the hard stop engages and keeps the
        /// axle from going below its minimum length, then the car settles normally
        void testHardStop() {
            createScene(Quaternion::identity(), decimal(3.0));

            bool hardStopSeen = false;
            decimal minLength = MAX_LENGTH;
            for (int i = 0; i < 360; i++) {
                mWorld->update(TIME_STEP);
                for (uint32 w = 0; w < 4; w++) {
                    const VehicleWheel& wheel = mVehicle->getWheel(w);
                    hardStopSeen |= wheel.hasHitHardStop();
                    minLength = std::min(minLength, wheel.getSuspensionLength());
                }
            }
            rp3d_test(hardStopSeen);
            // The hard stop is a constraint like the others: a little penetration is corrected,
            // a lot never happens
            rp3d_test(minLength > MIN_LENGTH - decimal(0.03));

            // Settled afterwards
            const decimal weight = MASS * GRAVITY;
            rp3d_test(std::abs(mVehicle->getTotalSuspensionForce(TIME_STEP) - weight) < weight * decimal(0.02));
            rp3d_test(up().y > decimal(0.9999));
            rp3d_test(mChassis->getLinearVelocity().length() < decimal(0.01));
            for (uint32 w = 0; w < 4; w++) {
                rp3d_test(!mVehicle->getWheel(w).hasHitHardStop());
            }

            destroyScene();
        }

        /// Destroying the chassis body takes the vehicle with it
        void testDestroyBodyDestroysVehicle() {
            createScene(Quaternion::identity(), decimal(1.3));
            step(5);

            mWorld->destroyRigidBody(mChassis);
            rp3d_test(mWorld->getNbVehicles() == 0);
            mVehicle = nullptr;

            // The world must still step fine without it
            step(5);

            mWorld->destroyRigidBody(mFloor);
            mPhysicsCommon.destroyPhysicsWorld(mWorld);
            mWorld = nullptr;
        }
};

}

#endif
