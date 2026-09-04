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
        // read (a kinematic platform moving under the car is felt by the suspension and tires).
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

// Cast the wheel rays and initialize the constraints before solving
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
                VehicleWheel& wheel = vehicle.mWheels[w];
                wheel.mSuspensionPart.deactivate();
                wheel.mHardStopPart.deactivate();
                wheel.mLongitudinalPart.deactivate();
                wheel.mLateralPart.deactivate();
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

// Find the ground under a wheel and set up its constraints
void SolveVehicleSystem::initWheel(VehicleConstraint& vehicle, VehicleWheel& wheel, const Transform& bodyTransform, const Vector3& worldUp) {

    const VehicleWheelSettings& settings = wheel.mSettings;
    const uint32 bodyIndex = vehicle.mBodyComponentIndex;

    // Spin the wheel: angular damping dw/dt = -c w integrated as w *= (1 - c dt), then the angle.
    // Contact or not, the wheel keeps turning; the longitudinal friction below is what couples it
    // to the ground.
    wheel.mAngularVelocity *= std::max(decimal(0.0), decimal(1.0) - settings.angularDamping * mTimeStep);
    wheel.mRotationAngle = std::fmod(wheel.mRotationAngle + wheel.mAngularVelocity * mTimeStep, decimal(2.0) * PI_RP3D);
    if (wheel.mRotationAngle < decimal(0.0)) wheel.mRotationAngle += decimal(2.0) * PI_RP3D;

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

        // In the air: full droop, no constraints
        wheel.mContactBody = nullptr;
        wheel.mSuspensionLength = settings.suspensionMaxLength;
        wheel.mSuspensionPart.deactivate();
        wheel.mHardStopPart.deactivate();
        wheel.mLongitudinalPart.deactivate();
        wheel.mLateralPart.deactivate();
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

    // The axle may never go below the plane through its current position (see the hard stop)
    wheel.mAxlePlaneConstant = wheel.mContactNormal.dot(anchorWorld + directionWorld * wheel.mSuspensionLength);

    // Tire basis in the contact plane: the rolling direction is the wheel forward direction
    // projected into the plane (built as normal x right so it stays perpendicular to the normal),
    // the lateral direction completes the frame, pointing to the right of the wheel
    const Vector3 forwardWorld = bodyTransform.getOrientation() * settings.wheelForward;
    const Vector3 rightWorld = bodyTransform.getOrientation() * settings.wheelUp.cross(settings.wheelForward);
    Vector3 longitudinal = wheel.mContactNormal.cross(rightWorld);
    if (longitudinal.dot(forwardWorld) < decimal(0.0)) longitudinal = -longitudinal;
    if (longitudinal.lengthSquare() > MACHINE_EPSILON) {
        longitudinal.normalize();
    }
    else {
        longitudinal = wheel.mContactNormal.getOneUnitOrthogonalVector();
    }
    wheel.mContactLongitudinal = longitudinal;
    wheel.mContactLateral = wheel.mContactNormal.cross(longitudinal).getUnit();

    // Where the tire forces act: at the contact point, or at a fixed point on the chassis
    const Vector3 forcePointWorld = settings.enableSuspensionForcePoint ? bodyTransform * settings.suspensionForcePoint : wheel.mContactPoint;
    wheel.mR2 = forcePointWorld - mRigidBodyComponents.mCentersOfMassWorld[bodyIndex];
    wheel.mR1 = wheel.mContactPoint - mRigidBodyComponents.mCentersOfMassWorld[wheel.mContactBodyComponentIndex];

    // Body 1 is the ground, body 2 the chassis: a positive impulse along the normal pushes the
    // chassis up off the ground, along the longitudinal axis it pushes the chassis forward
    const AxisConstraintBody ground = makeGroundBody(wheel);
    const AxisConstraintBody chassis = makeChassisBody(vehicle);

    // ---------- Suspension spring ---------- //

    const SpringSettings& spring = settings.suspensionSpring;
    if (settings.suspensionMaxLength > settings.suspensionMinLength && spring.isSoft()) {

        // Spring coefficients. In frequency mode they are derived from the effective mass of the
        // chassis at this wheel, computed once here from the LOCAL inertia along the vehicle's own
        // up axis at the neutral force point, so the stiffness does not change with the current
        // pitch, roll or contact normal of the car: the suspension feels the same on a slope.
        decimal stiffness, damping;
        bool hasSpring = true;
        if (spring.mode == SpringMode::FREQUENCY_AND_DAMPING_RATIO) {

            const Vector3 forcePointLocal = settings.enableSuspensionForcePoint ? settings.suspensionForcePoint :
                                            settings.position + directionLocal * (decimal(0.5) * (settings.suspensionMinLength + settings.suspensionMaxLength));
            const Vector3 r = forcePointLocal - mRigidBodyComponents.mCentersOfMassLocal[bodyIndex];
            const Vector3 rCrossNegUp = r.cross(-vehicle.mUp);
            const decimal inverseEffectiveMass = mRigidBodyComponents.mInverseMasses[bodyIndex] +
                                                 rCrossNegUp.dot(mRigidBodyComponents.mInverseInertiaTensorsLocal[bodyIndex] * rCrossNegUp);
            if (inverseEffectiveMass > MACHINE_EPSILON) {
                const decimal effectiveMass = decimal(1.0) / inverseEffectiveMass;
                const decimal omega = decimal(2.0) * PI_RP3D * spring.frequencyOrStiffness;
                stiffness = effectiveMass * omega * omega;
                damping = decimal(2.0) * effectiveMass * spring.damping * omega;
            }
            else {
                hasSpring = false;
                stiffness = damping = decimal(0.0);
            }
        }
        else {
            stiffness = spring.frequencyOrStiffness;
            damping = spring.damping;
        }

        if (hasSpring) {

            // The spring acts along the strut but the constraint acts along the contact normal.
            // With alpha the angle between them, F_normal = F_spring / cos(alpha), so k and c along
            // the normal are the strut values divided by cos(alpha). Clamped so a grazing contact
            // does not explode.
            const decimal cosAngle = std::max(decimal(0.1), directionWorld.dot(-wheel.mContactNormal));
            stiffness /= cosAngle;
            damping /= cosAngle;

            // Constraint error: negative when the suspension is compressed below its natural length
            const decimal positionError = wheel.mSuspensionLength - settings.suspensionMaxLength - settings.suspensionPreloadLength;

            wheel.mSuspensionPart.computeSpringConstraintProperties(mTimeStep, ground, wheel.mR1, chassis, wheel.mR2, wheel.mContactNormal,
                                                                    positionError, SpringSettings::fromStiffnessAndDamping(stiffness, damping));
        }
        else {
            wheel.mSuspensionPart.deactivate();
        }
    }
    else {
        wheel.mSuspensionPart.deactivate();
    }

    // ---------- Hard stop ---------- //

    // Once the suspension is fully compressed a hard constraint along the normal stops any
    // further approach (the position error is corrected in solvePositionConstraint)
    if (wheel.mSuspensionLength < settings.suspensionMinLength) {
        wheel.mHardStopPart.computeConstraintProperties(ground, wheel.mR1, chassis, wheel.mR2, wheel.mContactNormal);
    }
    else {
        wheel.mHardStopPart.deactivate();
    }

    // ---------- Tire friction ---------- //

    wheel.mLongitudinalPart.computeConstraintProperties(ground, wheel.mR1, chassis, wheel.mR2, wheel.mContactLongitudinal);
    wheel.mLateralPart.computeConstraintProperties(ground, wheel.mR1, chassis, wheel.mR2, wheel.mContactLateral);

    if (!mIsWarmStartingActive) {
        wheel.mSuspensionPart.resetTotalLambda();
        wheel.mHardStopPart.resetTotalLambda();
        wheel.mLongitudinalPart.resetTotalLambda();
        wheel.mLateralPart.resetTotalLambda();
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
            if (!wheel.hasContact()) continue;

            const AxisConstraintBody ground = makeGroundBody(wheel);
            wheel.mSuspensionPart.warmStart(ground, chassis, wheel.mContactNormal);
            wheel.mHardStopPart.warmStart(ground, chassis, wheel.mContactNormal);
            wheel.mLateralPart.warmStart(ground, chassis, wheel.mContactLateral);

            // The longitudinal impulse is NOT warm started: it is recomputed every step from the
            // difference between wheel spin and ground speed (and later the drive/brake torque),
            // and the spin of the wheel already carries the effect of last step's impulse.
            // Re-applying it would count it twice and act as a brake.
            wheel.mLongitudinalPart.resetTotalLambda();
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

        // Normal first, so the friction clamps below see this iteration's normal impulse
        solveSuspension(vehicle, chassis);
        solveLongitudinalFriction(vehicle, chassis);
        solveLateralFriction(vehicle, chassis);
    }
}

// Solve the suspension spring and hard stop of the wheels of a vehicle
void SolveVehicleSystem::solveSuspension(VehicleConstraint& vehicle, const AxisConstraintBody& chassis) {

    for (uint64 w = 0; w < vehicle.mWheels.size(); w++) {

        VehicleWheel& wheel = vehicle.mWheels[w];
        if (!wheel.hasContact()) continue;

        const AxisConstraintBody ground = makeGroundBody(wheel);

        // Both can only push the chassis away from the ground, never pull it down
        if (wheel.mSuspensionPart.isActive()) {
            wheel.mSuspensionPart.solveVelocityConstraint(ground, chassis, wheel.mContactNormal, decimal(0.0), DECIMAL_LARGEST);
        }
        if (wheel.mHardStopPart.isActive()) {
            wheel.mHardStopPart.solveVelocityConstraint(ground, chassis, wheel.mContactNormal, decimal(0.0), DECIMAL_LARGEST);
        }
    }
}

// Solve the longitudinal tire friction of the wheels of a vehicle (and update their spin)
void SolveVehicleSystem::solveLongitudinalFriction(VehicleConstraint& vehicle, const AxisConstraintBody& chassis) {

    for (uint64 w = 0; w < vehicle.mWheels.size(); w++) {

        VehicleWheel& wheel = vehicle.mWheels[w];
        if (!wheel.hasContact() || !wheel.mLongitudinalPart.isActive()) continue;

        const VehicleWheelSettings& settings = wheel.mSettings;
        const AxisConstraintBody ground = makeGroundBody(wheel);

        // The most the tire can transmit this step, from the normal impulse so far
        const decimal maxImpulse = settings.longitudinalFriction * wheel.getNormalImpulse();

        // Velocity of the chassis at the tire relative to the ground, along the rolling direction
        const Vector3 chassisPointVelocity = *chassis.linearVelocity + chassis.angularVelocity->cross(wheel.mR2);
        const Vector3 groundPointVelocity = *ground.linearVelocity + ground.angularVelocity->cross(wheel.mR1);
        const decimal relativeVelocity = wheel.mContactLongitudinal.dot(chassisPointVelocity - groundPointVelocity);

        // A free wheel rolls without slipping: apply the impulse that brings the surface speed of
        // the wheel to the ground speed in one step, limited by friction. The wheel spins up (or
        // down) by whatever impulse was actually applied, so a wheel that is friction-limited
        // keeps slipping instead of snapping to the ground speed.
        const decimal desiredAngularVelocity = relativeVelocity / settings.radius;
        const decimal impulseToMatch = (wheel.mAngularVelocity - desiredAngularVelocity) * settings.inertia / settings.radius;
        const decimal previousImpulse = wheel.mLongitudinalPart.getTotalLambda();
        const decimal targetImpulse = clamp(previousImpulse + impulseToMatch, -maxImpulse, maxImpulse);
        wheel.mLongitudinalPart.solveVelocityConstraint(ground, chassis, wheel.mContactLongitudinal, targetImpulse, targetImpulse);

        wheel.mAngularVelocity -= (wheel.mLongitudinalPart.getTotalLambda() - previousImpulse) * settings.radius / settings.inertia;
    }
}

// Solve the lateral tire friction of the wheels of a vehicle
void SolveVehicleSystem::solveLateralFriction(VehicleConstraint& vehicle, const AxisConstraintBody& chassis) {

    for (uint64 w = 0; w < vehicle.mWheels.size(); w++) {

        VehicleWheel& wheel = vehicle.mWheels[w];
        if (!wheel.hasContact() || !wheel.mLateralPart.isActive()) continue;

        const AxisConstraintBody ground = makeGroundBody(wheel);

        // Sideways the tire simply tries to stop all slip, up to its friction limit
        const decimal maxImpulse = wheel.mSettings.lateralFriction * wheel.getNormalImpulse();
        wheel.mLateralPart.solveVelocityConstraint(ground, chassis, wheel.mContactLateral, -maxImpulse, maxImpulse);
    }
}

// Solve the position constraints (hard stop of a fully compressed suspension)
void SolveVehicleSystem::solvePositionConstraint() {

    const uint64 nbVehicles = mVehicles.size();
    for (uint64 v = 0; v < nbVehicles; v++) {

        VehicleConstraint& vehicle = *mVehicles[v];
        if (!vehicle.mIsActiveThisStep) continue;

        const uint32 bodyIndex = vehicle.mBodyComponentIndex;
        Vector3& x2 = mRigidBodyComponents.mConstrainedPositions[bodyIndex];
        Quaternion& q2 = mRigidBodyComponents.mConstrainedOrientations[bodyIndex];
        const decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[bodyIndex];

        for (uint64 w = 0; w < vehicle.mWheels.size(); w++) {

            VehicleWheel& wheel = vehicle.mWheels[w];
            if (!wheel.hasContact()) continue;

            const VehicleWheelSettings& settings = wheel.mSettings;

            // Where the axle would be at minimum suspension length, given the corrected pose of
            // the chassis; it may not be below the axle plane recorded at contact time. (This
            // assumes the ground did not move during the step, like the joints assume for their
            // anchors.)
            const Vector3 directionLocal = settings.suspensionDirection.getUnit();
            const Vector3 axleAtMinLocal = settings.position + directionLocal * settings.suspensionMinLength;
            const Vector3 axleAtMinWorld = x2 + q2 * (axleAtMinLocal - mRigidBodyComponents.mCentersOfMassLocal[bodyIndex]);
            const decimal positionError = wheel.mContactNormal.dot(axleAtMinWorld) - wheel.mAxlePlaneConstant;
            if (positionError >= decimal(0.0)) continue;

            // Recompute the world inverse inertia tensor of the chassis from its corrected orientation
            Matrix3x3 inverseInertiaWorld;
            RigidBody::computeWorldInertiaTensorInverse(q2.getMatrix(), mRigidBodyComponents.mInverseInertiaTensorsLocal[bodyIndex], inverseInertiaWorld);

            // Push the chassis (and a dynamic ground) apart along the normal at the force point
            const Vector3& n = wheel.mContactNormal;
            const Vector3 r2 = axleAtMinWorld - x2;
            const Vector3 r2CrossN = r2.cross(n);
            const Vector3 invI2R2CrossN = inverseInertiaWorld * r2CrossN;
            decimal inverseMassMatrix = inverseMassBody2 + r2CrossN.dot(invI2R2CrossN);

            const bool groundIsDynamic = !wheel.mIsContactBodyFixed;
            const uint32 groundIndex = wheel.mContactBodyComponentIndex;
            Vector3 invI1R1CrossN(0, 0, 0);
            decimal inverseMassBody1 = decimal(0.0);
            if (groundIsDynamic) {
                inverseMassBody1 = mRigidBodyComponents.mInverseMasses[groundIndex];
                const Vector3 r1CrossN = wheel.mR1.cross(n);
                invI1R1CrossN = mRigidBodyComponents.mInverseInertiaTensorsWorld[groundIndex] * r1CrossN;
                inverseMassMatrix += inverseMassBody1 + r1CrossN.dot(invI1R1CrossN);
            }
            if (inverseMassMatrix <= MACHINE_EPSILON) continue;

            const decimal lambda = -positionError / inverseMassMatrix;

            // Chassis: pushed along +n
            const Vector3 v2 = inverseMassBody2 * mRigidBodyComponents.mLinearLockAxisFactors[bodyIndex] * (lambda * n);
            const Vector3 w2 = mRigidBodyComponents.mAngularLockAxisFactors[bodyIndex] * (lambda * invI2R2CrossN);
            x2 += v2;
            q2 += Quaternion(0, w2) * q2 * decimal(0.5);
            q2.normalize();

            // Dynamic ground: pushed along -n
            if (groundIsDynamic) {
                Vector3& x1 = mRigidBodyComponents.mConstrainedPositions[groundIndex];
                Quaternion& q1 = mRigidBodyComponents.mConstrainedOrientations[groundIndex];
                const Vector3 v1 = -inverseMassBody1 * mRigidBodyComponents.mLinearLockAxisFactors[groundIndex] * (lambda * n);
                const Vector3 w1 = -mRigidBodyComponents.mAngularLockAxisFactors[groundIndex] * (lambda * invI1R1CrossN);
                x1 += v1;
                q1 += Quaternion(0, w1) * q1 * decimal(0.5);
                q1.normalize();
            }
        }
    }
}
