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

// Constructor
VehicleWheel::VehicleWheel(const VehicleWheelSettings& settings)
             : mSettings(settings), mContactBody(nullptr), mContactBodyComponentIndex(0), mIsContactBodyFixed(true),
               mContactPoint(0, 0, 0), mContactNormal(0, 1, 0), mContactLongitudinal(0, 0, 1), mContactLateral(1, 0, 0),
               mR1(0, 0, 0), mR2(0, 0, 0), mAxlePlaneConstant(decimal(0.0)),
               mSuspensionLength(settings.suspensionMaxLength), mAngularVelocity(decimal(0.0)), mRotationAngle(decimal(0.0)) {

}

// Constructor
VehicleConstraint::VehicleConstraint(PhysicsWorld& world, RigidBody* body, const VehicleConstraintSettings& settings,
                                     MemoryAllocator& allocator)
                  : mWorld(world), mBody(body), mUp(settings.up.getUnit()), mForward(settings.forward.getUnit()),
                    mCosMaxSlopeAngle(std::cos(settings.maxSlopeAngle)), mRaycastCategoryMaskBits(settings.raycastCategoryMaskBits),
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
