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

// Libraries
#include <reactphysics3d/constraint/VehicleConstraint.h>
#include <reactphysics3d/body/RigidBody.h>
#include <reactphysics3d/engine/PhysicsWorld.h>

using namespace reactphysics3d;

// Rotation of `angle` radians about the unit vector `axis`
static Quaternion rotationAboutAxis(const Vector3& axis, decimal angle) {
    const decimal halfAngle = decimal(0.5) * angle;
    return Quaternion(axis * std::sin(halfAngle), std::cos(halfAngle));
}

// Constructor
VehicleWheel::VehicleWheel(const VehicleWheelSettings& settings)
             : mSettings(settings), mSteerAngle(decimal(0.0)), mDriveTorque(decimal(0.0)), mBrakeTorque(decimal(0.0)), mBrakeImpulse(decimal(0.0)),
               mContactBody(nullptr), mContactBodyComponentIndex(0), mIsContactBodyFixed(true),
               mContactPoint(0, 0, 0), mContactNormal(0, 1, 0), mContactLongitudinal(0, 0, 1), mContactLateral(-1, 0, 0),
               mR1(0, 0, 0), mR2(0, 0, 0), mAxlePlaneConstant(decimal(0.0)),
               mSuspensionLength(settings.suspensionMaxLength), mLateralSlipAngle(decimal(0.0)),
               mAngularVelocity(decimal(0.0)), mRotationAngle(decimal(0.0)) {

}

// Constructor
VehicleConstraint::VehicleConstraint(PhysicsWorld& world, RigidBody* body, const VehicleConstraintSettings& settings,
                                     MemoryAllocator& allocator)
                  : mWorld(world), mBody(body), mUp(settings.up.getUnit()), mForward(settings.forward.getUnit()),
                    mCosMaxSlopeAngle(std::cos(settings.maxSlopeAngle)), mRaycastCategoryMaskBits(settings.raycastCategoryMaskBits),
                    mWarmStartImpulseRatio(clamp(settings.warmStartImpulseRatio, decimal(0.0), decimal(1.0))),
                    mWheels(allocator), mBodyComponentIndex(0), mIsActiveThisStep(false) {

    assert(body != nullptr);
}

// Add a wheel and return its index
/**
 * @param settings Settings of the new wheel
 * @return Index of the wheel, for getWheel()
 */
uint32 VehicleConstraint::addWheel(const VehicleWheelSettings& settings) {

    assert(settings.suspensionMaxLength >= settings.suspensionMinLength);
    assert(settings.radius > decimal(0.0));
    assert(settings.inertia > decimal(0.0));

    mWheels.add(VehicleWheel(settings));
    return static_cast<uint32>(mWheels.size() - 1);
}

// Return the world-space centre of a wheel
/**
 * @param index Index of the wheel
 * @return Attachment point + current suspension length along the suspension direction, in world space
 */
Vector3 VehicleConstraint::getWheelCenterWorld(uint32 index) const {

    assert(index < mWheels.size());
    const VehicleWheel& wheel = mWheels[index];
    const Transform& transform = mBody->getTransform();
    const Vector3 anchor = transform * wheel.mSettings.position;
    const Vector3 direction = transform.getOrientation() * wheel.mSettings.suspensionDirection.getUnit();
    return anchor + direction * wheel.mSuspensionLength;
}

// Return the world-space transform of a wheel for rendering
/**
 * @param index Index of the wheel
 * @return Transform at the wheel centre, steered about the steering axis and rotated about the axle
 */
Transform VehicleConstraint::getWheelWorldTransform(uint32 index) const {

    assert(index < mWheels.size());
    const VehicleWheel& wheel = mWheels[index];
    const VehicleWheelSettings& settings = wheel.mSettings;

    // Roll about the axle (wheelUp x wheelForward: the left-pointing axle, so a positive angle
    // rolls the top of the wheel forward), then steer, then the chassis orientation
    const Vector3 axle = settings.wheelUp.cross(settings.wheelForward).getUnit();
    const Quaternion roll = rotationAboutAxis(axle, wheel.mRotationAngle);
    const Quaternion steer = rotationAboutAxis(settings.steeringAxis.getUnit(), wheel.mSteerAngle);
    const Quaternion orientation = mBody->getTransform().getOrientation() * steer * roll;

    return Transform(getWheelCenterWorld(index), orientation);
}

// Return the total force the suspension applied to the chassis this step
/**
 * @param timeStep The time step of the last simulation step
 * @return Sum over the wheels of the normal (spring + hard stop) impulse divided by the time step (N)
 */
decimal VehicleConstraint::getTotalSuspensionForce(decimal timeStep) const {

    assert(timeStep > MACHINE_EPSILON);
    decimal totalImpulse = decimal(0.0);
    for (uint64 i = 0; i < mWheels.size(); i++) {
        totalImpulse += mWheels[i].getNormalImpulse();
    }
    return totalImpulse / timeStep;
}
