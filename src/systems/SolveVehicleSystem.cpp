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
#include <cmath>

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

// Rotation of `angle` radians about the unit vector `axis`
Quaternion rotationAboutAxis(const Vector3& axis, decimal angle) {
    const decimal halfAngle = decimal(0.5) * angle;
    return Quaternion(axis * std::sin(halfAngle), std::cos(halfAngle));
}

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
                wheel.mLateralSlipAngle = decimal(0.0);
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

    // ---------- Spin the wheel ---------- //

    // Angular damping dw/dt = -c w, integrated as w *= (1 - c dt)
    wheel.mAngularVelocity *= std::max(decimal(0.0), decimal(1.0) - settings.angularDamping * mTimeStep);

    // The drive torque spins the wheel up; the tire then turns that spin into a push on the
    // road (or into wheelspin, if the tire cannot hold it)
    wheel.mAngularVelocity += wheel.mDriveTorque / settings.inertia * mTimeStep;

    // The brake slows the wheel towards a standstill, never past it. What it can transmit to the
    // road this step is the brake torque as an impulse at the tread.
    if (wheel.mBrakeTorque > decimal(0.0)) {
        const decimal brakeDeltaW = wheel.mBrakeTorque / settings.inertia * mTimeStep;
        if (std::abs(wheel.mAngularVelocity) <= brakeDeltaW) {
            wheel.mAngularVelocity = decimal(0.0);
        }
        else {
            wheel.mAngularVelocity -= (wheel.mAngularVelocity > decimal(0.0) ? brakeDeltaW : -brakeDeltaW);
        }
        wheel.mBrakeImpulse = wheel.mBrakeTorque * mTimeStep / settings.radius;
    }
    else {
        wheel.mBrakeImpulse = decimal(0.0);
    }

    // Contact or not, the wheel keeps turning
    wheel.mRotationAngle = std::fmod(wheel.mRotationAngle + wheel.mAngularVelocity * mTimeStep, decimal(2.0) * PI_RP3D);
    if (wheel.mRotationAngle < decimal(0.0)) wheel.mRotationAngle += decimal(2.0) * PI_RP3D;

    // ---------- Tire steer basis ---------- //

    // Steer the wheel forward and up directions about the steering axis. Needed both to offset
    // the ground samples along the (steered) rolling direction below, and to build the friction
    // basis once contact is found.
    const Quaternion steer = rotationAboutAxis(settings.steeringAxis.getUnit(), wheel.mSteerAngle);
    const Vector3 forwardLocal = steer * settings.wheelForward;
    const Vector3 upLocal = steer * settings.wheelUp;
    const Vector3 forwardWorld = bodyTransform.getOrientation() * forwardLocal;
    const Vector3 rightWorld = bodyTransform.getOrientation() * forwardLocal.cross(upLocal);

    // ---------- Find the ground ---------- //

    // Suspension ray(s): from the attachment point, along the suspension direction, as far as
    // the wheel can droop plus its radius (the tread touches down before the hub gets there).
    // With numContactSamples > 1, each extra ray is parallel to the first (same direction and
    // length) but offset forward/backward along the steered rolling direction by radius * sin of
    // an angle up to contactSampleHalfAngle - sample points around the tire's rim rather than
    // just its very bottom - and whichever hits soonest (smallest fraction) wins: that is the
    // part of the tire that would actually touch down first, e.g. against a kerb or bump edge
    // the single centre ray would still be short of. The offset is perpendicular to the ray
    // direction, so it does not affect how suspension length is measured from it below.
    const Vector3 directionLocal = settings.suspensionDirection.getUnit();
    const Vector3 directionWorld = bodyTransform.getOrientation() * directionLocal;
    const Vector3 anchorWorld = bodyTransform * settings.position;
    const decimal rayLength = settings.suspensionMaxLength + settings.radius;

    // A disabled wheel casts nothing: it is "in the air" whatever is under it
    const uint32 sampleCount = settings.enabled ? std::max<uint32>(1, settings.numContactSamples) : 0;
    WheelRaycastCallback callback(vehicle.mBody, worldUp, vehicle.mCosMaxSlopeAngle);
    for (uint32 s = 0; s < sampleCount; s++) {

        const decimal angle = sampleCount > 1 ?
            -settings.contactSampleHalfAngle + decimal(s) * (decimal(2.0) * settings.contactSampleHalfAngle) / decimal(sampleCount - 1) :
            decimal(0.0);
        const Vector3 sampleOrigin = anchorWorld + forwardWorld * (settings.radius * std::sin(angle));
        const Ray sampleRay(sampleOrigin, sampleOrigin + directionWorld * rayLength);

        WheelRaycastCallback sampleCallback(vehicle.mBody, worldUp, vehicle.mCosMaxSlopeAngle);
        mWorld.raycast(sampleRay, &sampleCallback, vehicle.mRaycastCategoryMaskBits);
        if (sampleCallback.hasHit && (!callback.hasHit || sampleCallback.fraction < callback.fraction)) {
            callback = sampleCallback;
        }
    }

    if (!callback.hasHit) {

        // In the air: full droop, no constraints
        wheel.mContactBody = nullptr;
        wheel.mSuspensionLength = settings.suspensionMaxLength;
        wheel.mSuspensionPart.deactivate();
        wheel.mHardStopPart.deactivate();
        wheel.mLongitudinalPart.deactivate();
        wheel.mLateralPart.deactivate();
        wheel.mLateralSlipAngle = decimal(0.0);
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

    // ---------- Tire basis ---------- //

    // Build the frame in the contact plane from the steer basis computed above: the rolling
    // direction is normal x right (so it stays perpendicular to the normal), flipped towards the
    // wheel forward direction; the lateral direction completes it, pointing to the right of the wheel
    Vector3 longitudinal = wheel.mContactNormal.cross(rightWorld);
    if (longitudinal.dot(forwardWorld) < decimal(0.0)) longitudinal = -longitudinal;
    if (longitudinal.lengthSquare() > MACHINE_EPSILON) {
        longitudinal.normalize();
    }
    else {
        longitudinal = wheel.mContactNormal.getOneUnitOrthogonalVector();
    }
    wheel.mContactLongitudinal = longitudinal;
    wheel.mContactLateral = longitudinal.cross(wheel.mContactNormal).getUnit();

    // Where the tire forces act: at the contact point, or at a fixed point on the chassis
    const Vector3 forcePointWorld = settings.enableSuspensionForcePoint ? bodyTransform * settings.suspensionForcePoint : wheel.mContactPoint;
    wheel.mR2 = forcePointWorld - mRigidBodyComponents.mCentersOfMassWorld[bodyIndex];
    wheel.mR1 = wheel.mContactPoint - mRigidBodyComponents.mCentersOfMassWorld[wheel.mContactBodyComponentIndex];

    // Body 1 is the ground, body 2 the chassis: a positive impulse along the normal pushes the
    // chassis up off the ground, along the longitudinal axis it pushes the chassis forward
    const AxisConstraintBody ground = makeGroundBody(wheel);
    const AxisConstraintBody chassis = makeChassisBody(vehicle);

    // ---------- Slip angle ---------- //

    // Where the tire is actually travelling relative to the ground under it. The angle between
    // that and the rolling direction is what builds the sideways force, and it is measured once
    // here rather than inside the solver so that every iteration works from the slip the step
    // started with instead of chasing the slip it has just removed.
    const Vector3 contactPointVelocity = *chassis.linearVelocity + chassis.angularVelocity->cross(wheel.mR2);
    const Vector3 groundPointVelocity = *ground.linearVelocity + ground.angularVelocity->cross(wheel.mR1);
    const Vector3 slipVelocity = contactPointVelocity - groundPointVelocity;
    wheel.mLateralSlipAngle = std::atan2(std::abs(wheel.mContactLateral.dot(slipVelocity)),
                                         std::abs(wheel.mContactLongitudinal.dot(slipVelocity)));

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

            // The spring is warm started in full: a soft constraint's impulse is determined by the
            // spring itself, so last step's value is simply the best guess. The hard parts (stop
            // and sideways friction) only partially: with several wheels on one chassis they can
            // hold an equal-and-opposite set of impulses that produces no velocity error, which
            // a full warm start would carry forward for ever - see
            // VehicleConstraintSettings::warmStartImpulseRatio.
            wheel.mSuspensionPart.warmStart(ground, chassis, wheel.mContactNormal);
            wheel.mHardStopPart.warmStart(ground, chassis, wheel.mContactNormal, vehicle.mWarmStartImpulseRatio);
            wheel.mLateralPart.warmStart(ground, chassis, wheel.mContactLateral, vehicle.mWarmStartImpulseRatio);

            // The longitudinal impulse is NOT warm started: it is recomputed every step from the
            // difference between wheel spin and ground speed (or from the brake), and the spin of
            // the wheel already carries the effect of last step's impulse. Re-applying it would
            // count it twice and act as a brake.
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

// How much of a tire's grip is left in one direction once the other direction has taken its
// share, as a fraction of that direction's own limit. Zero when the other direction is fully
// saturated, one when it carries nothing: the pair traces the friction ellipse. The two
// directions are solved one after the other and each sees the other's impulse from the previous
// solve, so over the velocity iterations they settle onto a shared split of the budget.
static decimal remainingGripFactor(decimal otherImpulse, decimal otherMaxImpulse) {

    if (otherMaxImpulse <= decimal(0.0)) return decimal(0.0);

    const decimal used = std::min(std::abs(otherImpulse) / otherMaxImpulse, decimal(1.0));
    return std::sqrt(std::max(decimal(0.0), decimal(1.0) - used * used));
}

// Solve the longitudinal tire friction of the wheels of a vehicle (and update their spin)
void SolveVehicleSystem::solveLongitudinalFriction(VehicleConstraint& vehicle, const AxisConstraintBody& chassis) {

    for (uint64 w = 0; w < vehicle.mWheels.size(); w++) {

        VehicleWheel& wheel = vehicle.mWheels[w];
        if (!wheel.hasContact() || !wheel.mLongitudinalPart.isActive()) continue;

        const VehicleWheelSettings& settings = wheel.mSettings;
        const AxisConstraintBody ground = makeGroundBody(wheel);

        // The most the tire can transmit this step, from the normal impulse so far, less whatever
        // share of the grip the sideways direction is already using
        const decimal maxFrictionImpulse = settings.longitudinalFriction * wheel.getNormalImpulse() *
                                           remainingGripFactor(wheel.mLateralPart.getTotalLambda(),
                                                               settings.lateralFriction * wheel.getNormalImpulse());

        // Velocity of the chassis at the tire relative to the ground, along the rolling direction
        const Vector3 chassisPointVelocity = *chassis.linearVelocity + chassis.angularVelocity->cross(wheel.mR2);
        const Vector3 groundPointVelocity = *ground.linearVelocity + ground.angularVelocity->cross(wheel.mR1);
        const decimal relativeVelocity = wheel.mContactLongitudinal.dot(chassisPointVelocity - groundPointVelocity);

        if (wheel.mBrakeImpulse > decimal(0.0)) {

            // Braking: stop the tread relative to the road, up to what the brake and the tire can
            // transmit, and never so much that the vehicle would be pushed the other way. The
            // wheel itself is slowed by the brake in initWheel(); a strong enough brake locks it
            // and the tire then slides at its friction limit.
            const decimal maxImpulse = std::min(wheel.mBrakeImpulse, maxFrictionImpulse);
            decimal minLambda, maxLambda;
            if (relativeVelocity >= decimal(0.0)) {
                minLambda = -maxImpulse;
                maxLambda = decimal(0.0);
            }
            else {
                minLambda = decimal(0.0);
                maxLambda = maxImpulse;
            }
            wheel.mLongitudinalPart.solveVelocityConstraint(ground, chassis, wheel.mContactLongitudinal, minLambda, maxLambda);
        }
        else {

            // Rolling: apply the impulse that brings the surface speed of the wheel to the ground
            // speed in one step, limited by friction. The wheel spins up (or down) by whatever
            // impulse was actually applied, so a driven wheel pushes the vehicle forward through
            // this coupling and a friction-limited one keeps slipping (wheelspin) instead of
            // snapping to the ground speed.
            const decimal desiredAngularVelocity = relativeVelocity / settings.radius;
            const decimal impulseToMatch = (wheel.mAngularVelocity - desiredAngularVelocity) * settings.inertia / settings.radius;
            const decimal previousImpulse = wheel.mLongitudinalPart.getTotalLambda();
            const decimal targetImpulse = clamp(previousImpulse + impulseToMatch, -maxFrictionImpulse, maxFrictionImpulse);
            wheel.mLongitudinalPart.solveVelocityConstraint(ground, chassis, wheel.mContactLongitudinal, targetImpulse, targetImpulse);

            wheel.mAngularVelocity -= (wheel.mLongitudinalPart.getTotalLambda() - previousImpulse) * settings.radius / settings.inertia;
        }
    }
}

// Solve the lateral tire friction of the wheels of a vehicle
void SolveVehicleSystem::solveLateralFriction(VehicleConstraint& vehicle, const AxisConstraintBody& chassis) {

    for (uint64 w = 0; w < vehicle.mWheels.size(); w++) {

        VehicleWheel& wheel = vehicle.mWheels[w];
        if (!wheel.hasContact() || !wheel.mLateralPart.isActive()) continue;

        const AxisConstraintBody ground = makeGroundBody(wheel);

        // Sideways the tire builds force with slip: cornering stiffness per radian of slip angle,
        // saturating at whatever grip is left once the rolling direction has taken its share. The
        // proportional band below saturation is what lets the sideways force rise and fall with
        // the heading error instead of being all or nothing.
        const VehicleWheelSettings& settings = wheel.mSettings;
        const decimal normalImpulse = wheel.getNormalImpulse();
        const decimal gripLimit = settings.lateralFriction * normalImpulse *
                                  remainingGripFactor(wheel.mLongitudinalPart.getTotalLambda(),
                                                      settings.longitudinalFriction * normalImpulse);
        const decimal maxImpulse = std::min(settings.corneringStiffness * wheel.mLateralSlipAngle * normalImpulse,
                                            gripLimit);
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
