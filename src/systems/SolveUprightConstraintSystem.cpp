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
#include <reactphysics3d/systems/SolveUprightConstraintSystem.h>
#include <reactphysics3d/constraint/UprightConstraint.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/body/RigidBody.h>

using namespace reactphysics3d;

// Static variables definition
const decimal SolveUprightConstraintSystem::LIMIT_TOLERANCE = decimal(0.001);

// Constructor
SolveUprightConstraintSystem::SolveUprightConstraintSystem(PhysicsWorld& world, RigidBodyComponents& rigidBodyComponents,
                                                           TransformComponents& transformComponents, Array<UprightConstraint*>& constraints)
                             : mWorld(world), mRigidBodyComponents(rigidBodyComponents), mTransformComponents(transformComponents),
                               mConstraints(constraints), mZeroInertia(Matrix3x3::zero()), mZeroVelocity(0, 0, 0),
                               mTimeStep(0), mIsWarmStartingActive(true) {

}

// Angle between the body axis and the world axis, and the righting axis
bool SolveUprightConstraintSystem::computeTilt(const Quaternion& orientation, const Vector3& localAxis, const Vector3& worldAxis,
                                               decimal& outAngle, Vector3& outRotationAxis) {

    const Vector3 bodyAxis = orientation * localAxis;
    const decimal cosAngle = clamp(bodyAxis.dot(worldAxis), decimal(-1.0), decimal(1.0));
    outAngle = std::acos(cosAngle);

    // Rotating the body about (bodyAxis x worldAxis) by the angle brings its axis onto the world axis
    Vector3 axis = bodyAxis.cross(worldAxis);
    if (axis.lengthSquare() > MACHINE_EPSILON) {
        axis.normalize();
        outRotationAxis = axis;
        return true;
    }

    // The two axes are parallel: upright (nothing to do) or exactly upside down (any axis
    // perpendicular to the world axis rights the body)
    if (cosAngle < decimal(0.0)) {
        outRotationAxis = worldAxis.getOneUnitOrthogonalVector();
        return true;
    }
    return false;
}

// Build the view of the world the part solves against
AxisConstraintBody SolveUprightConstraintSystem::makeWorldBody() {
    return AxisConstraintBody(&mZeroVelocity, &mZeroVelocity, decimal(0.0), &mZeroInertia, Vector3(0, 0, 0), Vector3(0, 0, 0));
}

// Build the view of the constrained body
AxisConstraintBody SolveUprightConstraintSystem::makeBody(const UprightConstraint& constraint) {

    const uint32 index = constraint.mBodyComponentIndex;
    return AxisConstraintBody(&mRigidBodyComponents.mConstrainedLinearVelocities[index],
                              &mRigidBodyComponents.mConstrainedAngularVelocities[index],
                              mRigidBodyComponents.mInverseMasses[index],
                              &mRigidBodyComponents.mInverseInertiaTensorsWorld[index],
                              mRigidBodyComponents.mLinearLockAxisFactors[index],
                              mRigidBodyComponents.mAngularLockAxisFactors[index]);
}

// Measure the tilt of each body and initialize the constraints before solving
void SolveUprightConstraintSystem::initBeforeSolve() {

    const uint64 nbConstraints = mConstraints.size();
    for (uint64 c = 0; c < nbConstraints; c++) {

        UprightConstraint& constraint = *mConstraints[c];
        const Entity bodyEntity = constraint.mBody->getEntity();

        // A sleeping or inactive body is not solved
        constraint.mIsActiveThisStep = !mRigidBodyComponents.getIsEntityDisabled(bodyEntity) &&
                                       mRigidBodyComponents.getBodyType(bodyEntity) == BodyType::DYNAMIC;
        if (!constraint.mIsActiveThisStep) {
            constraint.mPart.deactivate();
            continue;
        }

        constraint.mBodyComponentIndex = mRigidBodyComponents.getEntityIndex(bodyEntity);
        const UprightConstraintSettings& settings = constraint.mSettings;

        // How far the body leans, and which way to push it back
        decimal angle;
        Vector3 rotationAxis;
        const bool tilted = computeTilt(mTransformComponents.getTransform(bodyEntity).getOrientation(),
                                        settings.localAxis, settings.worldAxis, angle, rotationAxis);
        constraint.mCurrentAngle = angle;

        // Inside the cone (or exactly upright): free. A body sitting right AT the edge counts as
        // outside, like a joint at its limit: the position correction leaves it exactly on the
        // edge, and without this the velocity constraint would never get to remove the outward
        // spin that keeps pushing it over.
        if (!tilted || angle < settings.maxAngle - LIMIT_TOLERANCE) {
            constraint.mPart.deactivate();
            continue;
        }
        constraint.mRotationAxis = rotationAxis;

        // Angle error: negative by how much the body leans past the cone. A positive impulse about
        // the rotation axis reduces it.
        const decimal positionError = settings.maxAngle - angle;

        const AxisConstraintBody world = makeWorldBody();
        const AxisConstraintBody body = makeBody(constraint);
        if (settings.spring.isSoft()) {
            // Torsion spring-damper pulling the body back to the cone (bias handles the position error)
            constraint.mPart.computeSpringConstraintProperties(mTimeStep, world, body, rotationAxis, positionError, settings.spring);
        }
        else {
            // Hard stop: no bias here, the position error is corrected in solvePositionConstraint()
            constraint.mPart.computeConstraintProperties(world, body, rotationAxis);
        }

        if (!mIsWarmStartingActive) {
            constraint.mPart.resetTotalLambda();
        }
    }
}

// Warm start the constraints
void SolveUprightConstraintSystem::warmstart() {

    const uint64 nbConstraints = mConstraints.size();
    for (uint64 c = 0; c < nbConstraints; c++) {

        UprightConstraint& constraint = *mConstraints[c];
        if (!constraint.mIsActiveThisStep || !constraint.mPart.isActive()) continue;

        const AxisConstraintBody world = makeWorldBody();
        const AxisConstraintBody body = makeBody(constraint);
        constraint.mPart.warmStart(world, body);
    }
}

// Solve the velocity constraints
void SolveUprightConstraintSystem::solveVelocityConstraint() {

    const uint64 nbConstraints = mConstraints.size();
    for (uint64 c = 0; c < nbConstraints; c++) {

        UprightConstraint& constraint = *mConstraints[c];
        if (!constraint.mIsActiveThisStep || !constraint.mPart.isActive()) continue;

        const AxisConstraintBody world = makeWorldBody();
        const AxisConstraintBody body = makeBody(constraint);

        if (constraint.mPart.isSoft()) {
            // A spring-damper acts both ways: it also damps the swing back to upright
            constraint.mPart.solveVelocityConstraint(world, body, DECIMAL_SMALLEST, DECIMAL_LARGEST);
        }
        else {
            // A hard cone only ever pushes the body back inside, never holds it against the edge
            constraint.mPart.solveVelocityConstraint(world, body, decimal(0.0), DECIMAL_LARGEST);
        }
    }
}

// Solve the position constraints (hard cones only)
void SolveUprightConstraintSystem::solvePositionConstraint() {

    const uint64 nbConstraints = mConstraints.size();
    for (uint64 c = 0; c < nbConstraints; c++) {

        UprightConstraint& constraint = *mConstraints[c];
        if (!constraint.mIsActiveThisStep) continue;

        const UprightConstraintSettings& settings = constraint.mSettings;
        if (settings.spring.isSoft()) continue;

        const uint32 bodyIndex = constraint.mBodyComponentIndex;
        Quaternion& q = mRigidBodyComponents.mConstrainedOrientations[bodyIndex];

        // Re-measure the tilt from the corrected orientation
        decimal angle;
        Vector3 rotationAxis;
        if (!computeTilt(q, settings.localAxis, settings.worldAxis, angle, rotationAxis)) continue;
        const decimal positionError = settings.maxAngle - angle;
        if (positionError >= decimal(0.0)) continue;

        // Rotate the body back by the excess about the righting axis, weighted by its inertia
        // like the joints do: lambda = -C / K with K = n . I^-1 n
        Matrix3x3 inverseInertiaWorld;
        RigidBody::computeWorldInertiaTensorInverse(q.getMatrix(), mRigidBodyComponents.mInverseInertiaTensorsLocal[bodyIndex], inverseInertiaWorld);
        const Vector3 invIAxis = inverseInertiaWorld * rotationAxis;
        const decimal inverseMassMatrix = rotationAxis.dot(invIAxis);
        if (inverseMassMatrix <= MACHINE_EPSILON) continue;

        const decimal lambda = -positionError / inverseMassMatrix;
        const Vector3 w = mRigidBodyComponents.mAngularLockAxisFactors[bodyIndex] * (lambda * invIAxis);
        q += Quaternion(0, w) * q * decimal(0.5);
        q.normalize();
    }
}
