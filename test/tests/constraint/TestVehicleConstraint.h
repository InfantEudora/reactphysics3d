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
 * wheels resting their tread on the ground).
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

        // ---------- Methods ---------- //

        /// Create a floor (top surface through the origin, rotated by floorOrientation) and a
        /// 1000 kg box chassis with four wheels, spawned `height` above the floor along its normal
        void createScene(const Quaternion& floorOrientation, decimal height) {

            mWorld = mPhysicsCommon.createPhysicsWorld();
            mWorld->setGravity(Vector3(0, -GRAVITY, 0));

            const Vector3 floorNormal = floorOrientation * Vector3(0, 1, 0);

            // Floor: 20 x 1 x 20 box whose top face passes through the origin
            mFloor = mWorld->createRigidBody(Transform(floorNormal * decimal(-0.5), floorOrientation));
            mFloor->setType(BodyType::STATIC);
            BoxShape* floorShape = mPhysicsCommon.createBoxShape(Vector3(10, 0.5, 10));
            mFloorCollider = mFloor->addCollider(floorShape, Transform::identity());

            // Chassis: 1.6 x 0.5 x 3.6 box, 1000 kg
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
            testSlopeNormalForce();
            testDestroyBodyDestroysVehicle();
        }

        /// The vehicle is registered in the world and its wheels start at full droop
        void testCreation() {
            createScene(Quaternion::identity(), decimal(1.3));

            rp3d_test(mVehicle != nullptr);
            rp3d_test(mWorld->getNbVehicles() == 1);
            rp3d_test(mWorld->getVehicle(0) == mVehicle);
            rp3d_test(mVehicle->getBody() == mChassis);
            rp3d_test(mVehicle->getNbWheels() == 4);
            for (uint32 i = 0; i < 4; i++) {
                rp3d_test(!mVehicle->getWheel(i).hasContact());
                rp3d_test(approxEqual(mVehicle->getWheel(i).getSuspensionLength(), MAX_LENGTH));
                rp3d_test(approxEqual(mVehicle->getWheel(i).getSuspensionImpulse(), decimal(0.0)));
            }
            rp3d_test(Vector3::approxEqual(mVehicle->getLocalUp(), Vector3(0, 1, 0)));

            destroyScene();
        }

        /// Dropped onto flat ground the car settles level, with all four wheels touching, the
        /// suspension carrying exactly the weight and every tread resting on the floor
        void testRestOnFlatGround() {
            createScene(Quaternion::identity(), decimal(1.3));
            step(360);

            const decimal weight = MASS * GRAVITY;
            rp3d_test(std::abs(mVehicle->getTotalSuspensionForce(TIME_STEP) - weight) < weight * decimal(0.02));

            decimal minLength = MAX_LENGTH, maxLength = 0;
            for (uint32 i = 0; i < 4; i++) {
                const VehicleWheel& wheel = mVehicle->getWheel(i);
                rp3d_test(wheel.hasContact());
                rp3d_test(wheel.getContactBody() == mFloor);
                rp3d_test(Vector3::approxEqual(wheel.getContactNormal(), Vector3(0, 1, 0), decimal(1e-3)));
                rp3d_test(std::abs(wheel.getContactPoint().y) < decimal(1e-3));
                // Each wheel carries a quarter of the weight
                rp3d_test(std::abs(wheel.getSuspensionImpulse() / TIME_STEP - weight / decimal(4.0)) < weight * decimal(0.02));
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
            const Vector3 up = mChassis->getTransform().getOrientation() * Vector3(0, 1, 0);
            rp3d_test(up.y > decimal(0.9999));
            rp3d_test(mChassis->getLinearVelocity().length() < decimal(0.01));
            rp3d_test(mChassis->getAngularVelocity().length() < decimal(0.01));

            destroyScene();
        }

        /// A car far above the ground is in free fall: no contacts, no suspension force
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

        /// On a 10 degree slope (no tire friction yet, so the car slides) the suspension carries
        /// the weight component normal to the slope
        void testSlopeNormalForce() {
            const decimal angle = decimal(10.0) * PI_RP3D / decimal(180.0);
            createScene(Quaternion::fromEulerAngles(0, 0, angle), decimal(1.3));
            step(150);

            const decimal expected = MASS * GRAVITY * std::cos(angle);
            rp3d_test(std::abs(mVehicle->getTotalSuspensionForce(TIME_STEP) - expected) < expected * decimal(0.03));
            for (uint32 i = 0; i < 4; i++) {
                rp3d_test(mVehicle->getWheel(i).hasContact());
            }
            // Sliding down the slope (negative x is downhill for a positive rotation about z)
            rp3d_test(mChassis->getLinearVelocity().x < decimal(-0.5));

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
