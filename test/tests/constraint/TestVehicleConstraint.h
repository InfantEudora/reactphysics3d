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
 * wheels resting their tread on the ground), then rolling, sliding sideways, bottoming out,
 * driving, braking and steering.
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

        /// Mass equivalent of the four spinning wheels: 4 I / r^2
        static constexpr decimal WHEELS_MASS_EQUIVALENT = decimal(4.0) * WHEEL_INERTIA / (RADIUS * RADIUS);

        // ---------- Methods ---------- //

        /// Create a floor (top surface through the origin, rotated by floorOrientation) and a
        /// 1000 kg box chassis with four wheels, spawned `height` above the floor along its normal.
        /// Wheels 0 and 1 are the front (+z) left and right, 2 and 3 the rear.
        void createScene(const Quaternion& floorOrientation, decimal height) {

            mWorld = mPhysicsCommon.createPhysicsWorld();
            mWorld->setGravity(Vector3(0, -GRAVITY, 0));

            const Vector3 floorNormal = floorOrientation * Vector3(0, 1, 0);

            // Floor: 100 x 1 x 100 box whose top face passes through the origin
            mFloor = mWorld->createRigidBody(Transform(floorNormal * decimal(-0.5), floorOrientation));
            mFloor->setType(BodyType::STATIC);
            BoxShape* floorShape = mPhysicsCommon.createBoxShape(Vector3(50, 0.5, 50));
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
                wheel.position = Vector3((i % 2 == 0) ? 0.8 : -0.8, -0.2, (i < 2) ? 1.4 : -1.4);
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

        /// Right direction of the chassis in world space (forward x up = -x)
        Vector3 right() const {
            return mChassis->getTransform().getOrientation() * Vector3(-1, 0, 0);
        }

        /// Set the drive torque of the rear wheels
        void driveRear(decimal torque) {
            mVehicle->getWheel(2).setDriveTorque(torque);
            mVehicle->getWheel(3).setDriveTorque(torque);
        }

        /// Set the brake torque of all wheels
        void brakeAll(decimal torque) {
            for (uint32 i = 0; i < 4; i++) {
                mVehicle->getWheel(i).setBrakeTorque(torque);
            }
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
            testDrive();
            testWheelspin();
            testBrake();
            testSteering();
            testWheelTransform();
            testContactSamplingFlatGroundUnchanged();
            testContactSamplingFindsKerb();
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
                rp3d_test(approxEqual(wheel.getSteerAngle(), decimal(0.0)));
                rp3d_test(approxEqual(wheel.getDriveTorque(), decimal(0.0)));
                rp3d_test(approxEqual(wheel.getBrakeTorque(), decimal(0.0)));
            }
            rp3d_test(Vector3::approxEqual(mVehicle->getLocalUp(), Vector3(0, 1, 0)));
            rp3d_test(Vector3::approxEqual(mVehicle->getLocalForward(), Vector3(0, 0, 1)));

            // The steer angle is clamped to the max steer angle of the wheel
            mVehicle->getWheel(0).setSteerAngle(decimal(3.0));
            rp3d_test(approxEqual(mVehicle->getWheel(0).getSteerAngle(), mVehicle->getWheel(0).getSettings().maxSteerAngle));
            mVehicle->getWheel(0).setSteerAngle(decimal(0.0));

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
                // Right is forward x up = -x
                rp3d_test(Vector3::approxEqual(wheel.getContactLateral(), Vector3(-1, 0, 0), decimal(1e-3)));
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
            const decimal expectedAcceleration = GRAVITY * std::sin(angle) * MASS / (MASS + WHEELS_MASS_EQUIVALENT);
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
            const decimal expectedSpeed = decimal(5.0) * MASS / (MASS + WHEELS_MASS_EQUIVALENT);
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
            const decimal expectedDrift = forwardSpeed * WHEELS_MASS_EQUIVALENT / (MASS + WHEELS_MASS_EQUIVALENT);
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

        /// A torque on the rear wheels drives the car forward: a = (2 T / r) / (m + 4 I / r^2), all
        /// wheels rolling without slipping while the tires can hold the force
        void testDrive() {
            createScene(Quaternion::identity(), decimal(1.3));
            step(120);

            const decimal torque = decimal(300.0);
            driveRear(torque);
            step(120);

            const decimal expectedAcceleration = (decimal(2.0) * torque / RADIUS) / (MASS + WHEELS_MASS_EQUIVALENT);
            const decimal forwardSpeed = mChassis->getLinearVelocity().dot(forward());
            rp3d_test(std::abs(forwardSpeed - expectedAcceleration * decimal(2.0)) < expectedAcceleration * decimal(2.0) * decimal(0.05));
            rp3d_test(std::abs(mChassis->getLinearVelocity().dot(right())) < decimal(0.02));
            for (uint32 i = 0; i < 4; i++) {
                const VehicleWheel& wheel = mVehicle->getWheel(i);
                rp3d_test(wheel.hasContact());
                rp3d_test(std::abs(wheel.getAngularVelocity() * RADIUS - forwardSpeed) < forwardSpeed * decimal(0.05));
            }
            // The driven wheels push, the free wheels drag a little (they have to be spun up)
            rp3d_test(mVehicle->getWheel(2).getLongitudinalImpulse() > decimal(0.0));
            rp3d_test(mVehicle->getWheel(0).getLongitudinalImpulse() < decimal(0.0));

            // Letting go: the car coasts on
            driveRear(decimal(0.0));
            step(30);
            rp3d_test(mChassis->getLinearVelocity().dot(forward()) > forwardSpeed * decimal(0.95));

            destroyScene();
        }

        /// Far more torque than the tires can hold: the rear wheels spin faster than the ground
        /// and the car accelerates at the friction limit of the driven tires
        void testWheelspin() {
            createScene(Quaternion::identity(), decimal(1.3));
            step(120);

            driveRear(decimal(3000.0));
            step(60);

            const decimal forwardSpeed = mChassis->getLinearVelocity().dot(forward());
            // Friction-limited: roughly 2 mu N_rear / m, with N_rear about half the weight
            rp3d_test(forwardSpeed > decimal(3.5) && forwardSpeed < decimal(7.0));
            for (uint32 i = 2; i < 4; i++) {
                const VehicleWheel& wheel = mVehicle->getWheel(i);
                rp3d_test(wheel.getAngularVelocity() * RADIUS > decimal(1.5) * forwardSpeed);
                rp3d_test(std::abs(wheel.getLongitudinalImpulse() - wheel.getSettings().longitudinalFriction * wheel.getNormalImpulse()) < decimal(1e-3));
            }
            // The front wheels still roll with the ground
            for (uint32 i = 0; i < 2; i++) {
                rp3d_test(std::abs(mVehicle->getWheel(i).getAngularVelocity() * RADIUS - forwardSpeed) < forwardSpeed * decimal(0.05));
            }

            destroyScene();
        }

        /// Brakes: a gentle brake decelerates at 4 T / (r m), a hard one locks the wheels and
        /// stops the car at the friction limit
        void testBrake() {
            createScene(Quaternion::identity(), decimal(1.3));
            step(120);

            // Gentle: 300 N.m per wheel is 1000 N per wheel, 4 m/s^2
            mChassis->setLinearVelocity(Vector3(0, 0, 5));
            step(30);
            const decimal speedBefore = mChassis->getLinearVelocity().z;
            brakeAll(decimal(300.0));
            step(30);
            const decimal expectedDeceleration = decimal(4.0) * decimal(300.0) / RADIUS / MASS;
            rp3d_test(std::abs((speedBefore - mChassis->getLinearVelocity().z) / decimal(0.5) - expectedDeceleration) < expectedDeceleration * decimal(0.1));
            for (uint32 i = 0; i < 4; i++) {
                rp3d_test(mVehicle->getWheel(i).getLongitudinalImpulse() < decimal(0.0));
            }

            // Hard: 10000 N.m locks the wheels, the tires slide at their friction limit (mu = 1, so
            // about g) and 5 m/s is gone in about half a second
            brakeAll(decimal(0.0));
            mChassis->setLinearVelocity(Vector3(0, 0, 5));
            step(30);
            brakeAll(decimal(10000.0));
            step(60);
            rp3d_test(std::abs(mChassis->getLinearVelocity().z) < decimal(0.1));
            for (uint32 i = 0; i < 4; i++) {
                rp3d_test(std::abs(mVehicle->getWheel(i).getAngularVelocity()) < decimal(0.1));
            }
            // And it stays put, without creeping
            step(60);
            rp3d_test(std::abs(mChassis->getLinearVelocity().z) < decimal(0.02));

            destroyScene();
        }

        /// Steering the front wheels to the left while driving turns the car left (a positive yaw
        /// rate about the up axis), and the tire basis follows the steer angle
        void testSteering() {
            createScene(Quaternion::identity(), decimal(1.3));
            step(120);

            const decimal steer = decimal(20.0) * PI_RP3D / decimal(180.0);
            mVehicle->getWheel(0).setSteerAngle(steer);
            mVehicle->getWheel(1).setSteerAngle(steer);
            step(1);

            // Positive steer angle: the rolling direction rotates from +z towards +x (left)
            rp3d_test(Vector3::approxEqual(mVehicle->getWheel(0).getContactLongitudinal(), Vector3(std::sin(steer), 0, std::cos(steer)), decimal(1e-2)));
            rp3d_test(Vector3::approxEqual(mVehicle->getWheel(2).getContactLongitudinal(), Vector3(0, 0, 1), decimal(1e-2)));

            driveRear(decimal(300.0));
            step(180);

            // Turning left: yaw rate about +y is positive, and the car has moved to the left of its start
            rp3d_test(mChassis->getAngularVelocity().y > decimal(0.1));
            rp3d_test(mChassis->getTransform().getPosition().x > decimal(0.5));
            rp3d_test(mChassis->getLinearVelocity().dot(forward()) > decimal(1.0));
            rp3d_test(up().y > decimal(0.99));

            // Straighten up: the yaw rate dies down
            mVehicle->getWheel(0).setSteerAngle(decimal(0.0));
            mVehicle->getWheel(1).setSteerAngle(decimal(0.0));
            step(60);
            rp3d_test(std::abs(mChassis->getAngularVelocity().y) < decimal(0.05));

            destroyScene();
        }

        /// The wheel transform sits at the wheel centre, steers with the wheel and rolls forward
        /// for a positive rotation angle
        void testWheelTransform() {
            createScene(Quaternion::identity(), decimal(1.3));
            step(120);

            VehicleWheel& wheel = mVehicle->getWheel(0);
            wheel.setSteerAngle(decimal(0.0));
            wheel.setRotationAngle(decimal(0.0));
            Transform transform = mVehicle->getWheelWorldTransform(0);
            rp3d_test(Vector3::approxEqual(transform.getPosition(), mVehicle->getWheelCenterWorld(0), decimal(1e-5)));
            rp3d_test(Vector3::approxEqual(transform.getOrientation() * Vector3(0, 0, 1), Vector3(0, 0, 1), decimal(1e-3)));

            // A quarter turn forward brings the top of the wheel to the front
            wheel.setRotationAngle(decimal(0.5) * PI_RP3D);
            transform = mVehicle->getWheelWorldTransform(0);
            rp3d_test(Vector3::approxEqual(transform.getOrientation() * Vector3(0, 1, 0), Vector3(0, 0, 1), decimal(1e-3)));
            // The axle is unchanged by rolling
            rp3d_test(Vector3::approxEqual(transform.getOrientation() * Vector3(1, 0, 0), Vector3(1, 0, 0), decimal(1e-3)));

            // Steering to the left turns the forward direction towards +x (60 degrees: within the
            // default 70 degree lock the angle is clamped to)
            const decimal steer = decimal(60.0) * PI_RP3D / decimal(180.0);
            wheel.setRotationAngle(decimal(0.0));
            wheel.setSteerAngle(steer);
            transform = mVehicle->getWheelWorldTransform(0);
            rp3d_test(Vector3::approxEqual(transform.getOrientation() * Vector3(0, 0, 1), Vector3(std::sin(steer), 0, std::cos(steer)), decimal(1e-3)));
            rp3d_test(Vector3::approxEqual(transform.getOrientation() * Vector3(0, 1, 0), Vector3(0, 1, 0), decimal(1e-3)));

            destroyScene();
        }

        /// numContactSamples > 1 fans the wheel ray into several, each offset forward/backward
        /// along the rolling direction by radius * sin(angle). On flat ground every sample lands
        /// on the same plane, so the result must be unchanged from a single ray: same contact
        /// body, suspension length and normal impulse, well within tolerance of each other.
        void testContactSamplingFlatGroundUnchanged() {
            createScene(Quaternion::identity(), decimal(1.3));
            for (uint32 i = 0; i < 4; i++) {
                mVehicle->getWheel(i).getSettings().numContactSamples = 5;
            }
            step(360);

            const decimal weight = MASS * GRAVITY;
            rp3d_test(std::abs(mVehicle->getTotalSuspensionForce(TIME_STEP) - weight) < weight * decimal(0.02));
            for (uint32 i = 0; i < 4; i++) {
                const VehicleWheel& wheel = mVehicle->getWheel(i);
                rp3d_test(wheel.hasContact());
                rp3d_test(wheel.getContactBody() == mFloor);
                rp3d_test(Vector3::approxEqual(wheel.getContactNormal(), Vector3(0, 1, 0), decimal(1e-3)));
                rp3d_test(std::abs(wheel.getSuspensionLength() - (MAX_LENGTH + MIN_LENGTH) * decimal(0.5)) < decimal(0.02));
            }
            rp3d_test(up().y > decimal(0.9999));

            destroyScene();
        }

        /// A low kerb sits just ahead of wheel 0's own vertical line: within radius * sin(45 deg)
        /// (about 0.212 m) of it, so an outer sample of a 3-way fan reaches it, but outside the
        /// single centre ray's own line, so that ray never notices it at all. This is exactly the
        /// improved bump/kerb handling multiple samples are for.
        void testContactSamplingFindsKerb() {

            // A single centre ray: finds only the plain floor beneath the anchor
            createScene(Quaternion::identity(), decimal(0.95));
            RigidBody* kerb = mWorld->createRigidBody(Transform(Vector3(0.8, 0.1, 1.65), Quaternion::identity()));
            kerb->setType(BodyType::STATIC);
            kerb->addCollider(mPhysicsCommon.createBoxShape(Vector3(0.15, 0.1, 0.15)), Transform::identity());
            step(1);

            rp3d_test(mVehicle->getWheel(0).hasContact());
            rp3d_test(mVehicle->getWheel(0).getContactBody() == mFloor);
            rp3d_test(std::abs(mVehicle->getWheel(0).getSuspensionLength() - decimal(0.45)) < decimal(0.01));

            mWorld->destroyRigidBody(kerb);
            destroyScene();

            // The same scene again, wheel 0 now sampling a 3-way fan: the outermost sample lands
            // on the kerb, closer than the plain floor, so that is what the wheel finds instead
            createScene(Quaternion::identity(), decimal(0.95));
            RigidBody* kerb2 = mWorld->createRigidBody(Transform(Vector3(0.8, 0.1, 1.65), Quaternion::identity()));
            kerb2->setType(BodyType::STATIC);
            kerb2->addCollider(mPhysicsCommon.createBoxShape(Vector3(0.15, 0.1, 0.15)), Transform::identity());
            mVehicle->getWheel(0).getSettings().numContactSamples = 3;
            step(1);

            rp3d_test(mVehicle->getWheel(0).hasContact());
            rp3d_test(mVehicle->getWheel(0).getContactBody() == kerb2);
            rp3d_test(std::abs(mVehicle->getWheel(0).getSuspensionLength() - decimal(0.25)) < decimal(0.01));

            // The other three wheels, unchanged and far from the kerb, still just see the floor
            for (uint32 i = 1; i < 4; i++) {
                rp3d_test(mVehicle->getWheel(i).getContactBody() == mFloor);
            }

            mWorld->destroyRigidBody(kerb2);
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
