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
#include <reactphysics3d/systems/SolveVehicleSystem.h>
#include <reactphysics3d/constraint/VehicleConstraint.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/body/RigidBody.h>
#include <reactphysics3d/collision/RaycastInfo.h>
#include <reactphysics3d/collision/Collider.h>

using namespace reactphysics3d;

namespace {

// Nearest ground hit for a wheel ray: ignores the chassis itself, trigger colliders and
// surfaces steeper than the vehicle's max slope angle (a wall is not a road)
class WheelRaycastCallback : public RaycastCallback {

    public :

        const Body* excludedBody;
        Vector3 worldUp;
        decimal cosMaxSlopeAngle;

        bool hasHit;
        Body* body;
        Vector3 point;
        Vector3 normal;
        decimal fraction;

        WheelRaycastCallback(const Body* excluded, const Vector3& up, decimal cosMaxSlope)
            : excludedBody(excluded), worldUp(up), cosMaxSlopeAngle(cosMaxSlope), hasHit(false), body(nullptr),
              point(0, 0, 0), normal(0, 1, 0), fraction(decimal(1.0)) {}

        virtual decimal notifyRaycastHit(const RaycastInfo& info) override {

            // -1 ignores this collider and continues the ray unclipped
            if (info.body == excludedBody) return decimal(-1.0);
            if (info.collider->getIsTrigger()) return decimal(-1.0);
            if (info.worldNormal.dot(worldUp) < cosMaxSlopeAngle) return decimal(-1.0);

            hasHit = true;
            body = info.body;
            point = info.worldPoint;
            normal = info.worldNormal;
            fraction = info.hitFraction;

            // Clip the ray so only nearer hits are reported from now on
            return info.hitFraction;
        }
};

}

// Constructor
SolveVehicleSystem::SolveVehicleSystem(PhysicsWorld& world, RigidBodyComponents& rigidBodyComponents,
                                       TransformComponents& transformComponents, Array<VehicleConstraint*>& vehicles)
                   : mWorld(world), mRigidBodyComponents(rigidBodyComponents), mTransformComponents(transformComponents),
                     mVehicles(vehicles), mZeroInertia(Matrix3x3::zero()), mTimeStep(0), mIsWarmStartingActive(true) {

}

// Build the view of the chassis the AxisConstraintPart solves against
AxisConstraintBody SolveVehicleSystem::makeChassisBody(const VehicleConstraint& vehicle) {

    const uint32 index = vehicle.mBodyComponentIndex;
    return AxisConstraintBody(&mRigidBodyComponents.mConstrainedLinearVelocities[index],
                              &mRigidBodyComponents.mConstrainedAngularVelocities[index],
                              mRigidBodyComponents.mInverseMasses[index],
                              &mRigidBodyComponents.mInverseInertiaTensorsWorld[index],
                              mRigidBodyComponents.mLinearLockAxisFactors[index],
                              mRigidBodyComponents.mAngularLockAxisFactors[index]);
}

// Build the view of the ground body under a wheel
AxisConstraintBody SolveVehicleSystem::makeGroundBody(const VehicleWheel& wheel) {

    const uint32 index = wheel.mContactBodyComponentIndex;

    if (wheel.mIsContactBodyFixed) {
        // Static, kinematic or sleeping ground: it takes no impulse. Its velocities are still
        // read (a kinematic platform moving under the car is felt by the suspension).
        return AxisConstraintBody(&mRigidBodyComponents.mConstrainedLinearVelocities[index],
                                  &mRigidBodyComponents.mConstrainedAngularVelocities[index],
                                  decimal(0.0), &mZeroInertia);
    }

    return AxisConstraintBody(&mRigidBodyComponents.mConstrainedLinearVelocities[index],
                              &mRigidBodyComponents.mConstrainedAngularVelocities[index],
                              mRigidBodyComponents.mInverseMasses[index],
                              &mRigidBodyComponents.mInverseInertiaTensorsWorld[index],
                              mRigidBodyComponents.mLinearLockAxisFactors[index],
                              mRigidBodyComponents.mAngularLockAxisFactors[index]);
}

// Cast the wheel rays and initialize the suspension constraints before solving
void SolveVehicleSystem::initBeforeSolve() {

    // World up from gravity, used to tell ground from walls
    Vector3 worldUp = -mWorld.getGravity();
    if (worldUp.lengthSquare() > MACHINE_EPSILON) {
        worldUp.normalize();
    }
    else {
        worldUp.setAllValues(0, 1, 0);
    }

    const uint64 nbVehicles = mVehicles.size();
    for (uint64 v = 0; v < nbVehicles; v++) {

        VehicleConstraint& vehicle = *mVehicles[v];
        const Entity bodyEntity = vehicle.mBody->getEntity();

        // A sleeping or inactive chassis is not solved (its velocities are not integrated either)
        vehicle.mIsActiveThisStep = !mRigidBodyComponents.getIsEntityDisabled(bodyEntity) &&
                                    mRigidBodyComponents.getBodyType(bodyEntity) == BodyType::DYNAMIC;
        if (!vehicle.mIsActiveThisStep) {
            for (uint64 w = 0; w < vehicle.mWheels.size(); w++) {
                vehicle.mWheels[w].mSuspensionPart.deactivate();
            }
            continue;
        }

        vehicle.mBodyComponentIndex = mRigidBodyComponents.getEntityIndex(bodyEntity);
        const Transform& bodyTransform = mTransformComponents.getTransform(bodyEntity);

        for (uint64 w = 0; w < vehicle.mWheels.size(); w++) {
            initWheel(vehicle, vehicle.mWheels[w], bodyTransform, worldUp);
        }
    }
}

// Find the ground under a wheel and set up its suspension constraint
void SolveVehicleSystem::initWheel(VehicleConstraint& vehicle, VehicleWheel& wheel, const Transform& bodyTransform, const Vector3& worldUp) {

    const VehicleWheelSettings& settings = wheel.mSettings;
    const uint32 bodyIndex = vehicle.mBodyComponentIndex;

    // Suspension ray: from the attachment point, along the suspension direction, as far as the
    // wheel can droop plus its radius (the tread touches down before the hub gets there)
    const Vector3 directionLocal = settings.suspensionDirection.getUnit();
    const Vector3 directionWorld = bodyTransform.getOrientation() * directionLocal;
    const Vector3 anchorWorld = bodyTransform * settings.position;
    const decimal rayLength = settings.suspensionMaxLength + settings.radius;
    const Ray ray(anchorWorld, anchorWorld + directionWorld * rayLength);

    WheelRaycastCallback callback(vehicle.mBody, worldUp, vehicle.mCosMaxSlopeAngle);
    mWorld.raycast(ray, &callback, vehicle.mRaycastCategoryMaskBits);

    if (!callback.hasHit) {

        // In the air: full droop, no constraint
        wheel.mContactBody = nullptr;
        wheel.mSuspensionLength = settings.suspensionMaxLength;
        wheel.mSuspensionPart.deactivate();
        return;
    }

    // Contact
    const Entity contactEntity = callback.body->getEntity();
    wheel.mContactBody = static_cast<RigidBody*>(callback.body);
    wheel.mContactBodyComponentIndex = mRigidBodyComponents.getEntityIndex(contactEntity);
    wheel.mIsContactBodyFixed = mRigidBodyComponents.getBodyType(contactEntity) != BodyType::DYNAMIC ||
                                mRigidBodyComponents.getIsEntityDisabled(contactEntity);
    wheel.mContactPoint = callback.point;
    wheel.mContactNormal = callback.normal;
    wheel.mSuspensionLength = std::max(decimal(0.0), rayLength * callback.fraction - settings.radius);

    // No travel or no spring: nothing to solve here (the hard stop is a separate part)
    const SpringSettings& spring = settings.suspensionSpring;
    if (settings.suspensionMaxLength <= settings.suspensionMinLength || !spring.isSoft()) {
        wheel.mSuspensionPart.deactivate();
        return;
    }

    // Spring coefficients. In frequency mode they are derived from the effective mass of the
    // chassis at this wheel, computed once here from the LOCAL inertia along the vehicle's own up
    // axis at the neutral force point, so the stiffness does not change with the current pitch,
    // roll or contact normal of the car: the suspension feels the same on a slope as on the flat.
    decimal stiffness, damping;
    if (spring.mode == SpringMode::FREQUENCY_AND_DAMPING_RATIO) {

        const Vector3 forcePointLocal = settings.enableSuspensionForcePoint ? settings.suspensionForcePoint :
                                        settings.position + directionLocal * (decimal(0.5) * (settings.suspensionMinLength + settings.suspensionMaxLength));
        const Vector3 r = forcePointLocal - mRigidBodyComponents.mCentersOfMassLocal[bodyIndex];
        const Vector3 rCrossNegUp = r.cross(-vehicle.mUp);
        const decimal inverseEffectiveMass = mRigidBodyComponents.mInverseMasses[bodyIndex] +
                                             rCrossNegUp.dot(mRigidBodyComponents.mInverseInertiaTensorsLocal[bodyIndex] * rCrossNegUp);
        if (inverseEffectiveMass <= MACHINE_EPSILON) {
            wheel.mSuspensionPart.deactivate();
            return;
        }
        const decimal effectiveMass = decimal(1.0) / inverseEffectiveMass;
        const decimal omega = decimal(2.0) * PI_RP3D * spring.frequencyOrStiffness;
        stiffness = effectiveMass * omega * omega;
        damping = decimal(2.0) * effectiveMass * spring.damping * omega;
    }
    else {
        stiffness = spring.frequencyOrStiffness;
        damping = spring.damping;
    }

    // The spring acts along the strut but the constraint acts along the contact normal. With
    // alpha the angle between them, F_normal = F_spring / cos(alpha), so k and c along the normal
    // are the strut values divided by cos(alpha). Clamped so a grazing contact does not explode.
    const decimal cosAngle = std::max(decimal(0.1), directionWorld.dot(-wheel.mContactNormal));
    stiffness /= cosAngle;
    damping /= cosAngle;

    // Body 1 is the ground, body 2 the chassis, the axis is the contact normal: a positive impulse
    // pushes the chassis up off the ground
    const Vector3 forcePointWorld = settings.enableSuspensionForcePoint ? bodyTransform * settings.suspensionForcePoint : wheel.mContactPoint;
    const Vector3 r2 = forcePointWorld - mRigidBodyComponents.mCentersOfMassWorld[bodyIndex];
    const Vector3 r1 = wheel.mContactPoint - mRigidBodyComponents.mCentersOfMassWorld[wheel.mContactBodyComponentIndex];

    // Constraint error: negative when the suspension is compressed below its natural length
    const decimal positionError = wheel.mSuspensionLength - settings.suspensionMaxLength - settings.suspensionPreloadLength;

    const AxisConstraintBody ground = makeGroundBody(wheel);
    const AxisConstraintBody chassis = makeChassisBody(vehicle);
    wheel.mSuspensionPart.computeSpringConstraintProperties(mTimeStep, ground, r1, chassis, r2, wheel.mContactNormal, positionError,
                                                            SpringSettings::fromStiffnessAndDamping(stiffness, damping));

    if (!mIsWarmStartingActive) {
        wheel.mSuspensionPart.resetTotalLambda();
    }
}

// Warm start the constraints (apply the previous impulses at the beginning of the step)
void SolveVehicleSystem::warmstart() {

    const uint64 nbVehicles = mVehicles.size();
    for (uint64 v = 0; v < nbVehicles; v++) {

        VehicleConstraint& vehicle = *mVehicles[v];
        if (!vehicle.mIsActiveThisStep) continue;

        const AxisConstraintBody chassis = makeChassisBody(vehicle);
        for (uint64 w = 0; w < vehicle.mWheels.size(); w++) {

            VehicleWheel& wheel = vehicle.mWheels[w];
            if (!wheel.mSuspensionPart.isActive()) continue;

            const AxisConstraintBody ground = makeGroundBody(wheel);
            wheel.mSuspensionPart.warmStart(ground, chassis, wheel.mContactNormal);
        }
    }
}

// Solve the velocity constraints
void SolveVehicleSystem::solveVelocityConstraint() {

    const uint64 nbVehicles = mVehicles.size();
    for (uint64 v = 0; v < nbVehicles; v++) {

        VehicleConstraint& vehicle = *mVehicles[v];
        if (!vehicle.mIsActiveThisStep) continue;

        const AxisConstraintBody chassis = makeChassisBody(vehicle);
        for (uint64 w = 0; w < vehicle.mWheels.size(); w++) {

            VehicleWheel& wheel = vehicle.mWheels[w];
            if (!wheel.mSuspensionPart.isActive()) continue;

            // The spring can only push the chassis away from the ground, never pull it down
            const AxisConstraintBody ground = makeGroundBody(wheel);
            wheel.mSuspensionPart.solveVelocityConstraint(ground, chassis, wheel.mContactNormal, decimal(0.0), DECIMAL_LARGEST);
        }
    }
}

// Solve the position constraints
void SolveVehicleSystem::solvePositionConstraint() {

    // The suspension is a soft constraint: its bias term corrects the position error, there is
    // nothing to do here. The hard stop at minimum suspension length will need this later.
}
