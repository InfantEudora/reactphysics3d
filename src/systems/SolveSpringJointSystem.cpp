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
#include <reactphysics3d/systems/SolveSpringJointSystem.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/body/RigidBody.h>

using namespace reactphysics3d;

// Static variables definition
const decimal SolveSpringJointSystem::BETA = decimal(0.2);

// Constructor
SolveSpringJointSystem::SolveSpringJointSystem(PhysicsWorld& world, RigidBodyComponents& rigidBodyComponents,
                                               TransformComponents& transformComponents,
                                               JointComponents& jointComponents,
                                               SpringJointComponents& springJointComponents)
              :mWorld(world), mRigidBodyComponents(rigidBodyComponents), mTransformComponents(transformComponents),
               mJointComponents(jointComponents), mSpringJointComponents(springJointComponents),
               mTimeStep(0), mIsWarmStartingActive(true) {

}

// Initialize before solving the constraint
void SolveSpringJointSystem::initBeforeSolve() {

    const decimal biasFactor = (BETA / mTimeStep);

    // For each joint
    const uint32 nbJoints = mSpringJointComponents.getNbEnabledComponents();
    for (uint32 i=0; i < nbJoints; i++) {

        const Entity jointEntity = mSpringJointComponents.mJointEntities[i];
        const uint32 jointIndex = mJointComponents.getEntityIndex(jointEntity);

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.mBody1Entities[jointIndex];
        const Entity body2Entity = mJointComponents.mBody2Entities[jointIndex];

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        assert(!mRigidBodyComponents.getIsEntityDisabled(body1Entity) || !mRigidBodyComponents.getIsEntityDisabled(body2Entity));

        // Get the inertia tensor of bodies
        mSpringJointComponents.mI1[i] = mRigidBodyComponents.mInverseInertiaTensorsWorld[componentIndexBody1];
        mSpringJointComponents.mI2[i] = mRigidBodyComponents.mInverseInertiaTensorsWorld[componentIndexBody2];

        const Quaternion& orientationBody1 = mTransformComponents.getTransform(body1Entity).getOrientation();
        const Quaternion& orientationBody2 = mTransformComponents.getTransform(body2Entity).getOrientation();

        // Vector from body center of mass to the anchor point
        mSpringJointComponents.mR1World[i] = orientationBody1 * (mSpringJointComponents.mLocalAnchorPointBody1[i] - mRigidBodyComponents.mCentersOfMassLocal[componentIndexBody1]);
        mSpringJointComponents.mR2World[i] = orientationBody2 * (mSpringJointComponents.mLocalAnchorPointBody2[i] - mRigidBodyComponents.mCentersOfMassLocal[componentIndexBody2]);

        const Vector3& x1 = mRigidBodyComponents.mCentersOfMassWorld[componentIndexBody1];
        const Vector3& x2 = mRigidBodyComponents.mCentersOfMassWorld[componentIndexBody2];
        const Vector3& r1 = mSpringJointComponents.mR1World[i];
        const Vector3& r2 = mSpringJointComponents.mR2World[i];

        // Vector between the two anchors: the constraint axis points from anchor 1 to anchor 2
        const Vector3 u = x2 + r2 - x1 - r1;
        const decimal length = u.length();
        mSpringJointComponents.mCurrentLength[i] = length;
        if (length > MACHINE_EPSILON) {
            mSpringJointComponents.mAxisWorld[i] = u / length;
        }
        // else: the anchors coincide and there is no direction to push along; keep the last axis

        // Constraint error: positive when the spring is stretched
        const decimal positionError = length - mSpringJointComponents.mRestLength[i];
        const SpringSettings& spring = mSpringJointComponents.mSpringSettings[i];

        // A rigid rod (no spring) corrects its position error either here through a Baumgarte
        // bias, or in solvePositionConstraint() (non-linear Gauss-Seidel). A soft spring has
        // its own bias built in and never needs position correction.
        decimal bias = decimal(0.0);
        if (!spring.isSoft() && mJointComponents.mPositionCorrectionTechniques[jointIndex] == JointsPositionCorrectionTechnique::BAUMGARTE_JOINTS) {
            bias = biasFactor * positionError;
        }

        const AxisConstraintBody body1 = makeConstraintBody(componentIndexBody1, &mSpringJointComponents.mI1[i]);
        const AxisConstraintBody body2 = makeConstraintBody(componentIndexBody2, &mSpringJointComponents.mI2[i]);

        AxisConstraintPart& part = mSpringJointComponents.mConstraintPart[i];
        part.computeSpringConstraintProperties(mTimeStep, body1, r1, body2, r2, mSpringJointComponents.mAxisWorld[i],
                                               positionError, spring, bias);

        // If warm-starting is not enabled
        if (!mIsWarmStartingActive) {

            // Reset the accumulated impulse
            part.resetTotalLambda();
        }
    }
}

// Warm start the constraint (apply the previous impulse at the beginning of the step)
void SolveSpringJointSystem::warmstart() {

    // For each joint component
    const uint32 nbJoints = mSpringJointComponents.getNbEnabledComponents();
    for (uint32 i=0; i < nbJoints; i++) {

        const Entity jointEntity = mSpringJointComponents.mJointEntities[i];
        const uint32 jointIndex = mJointComponents.getEntityIndex(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(mJointComponents.mBody1Entities[jointIndex]);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(mJointComponents.mBody2Entities[jointIndex]);

        const AxisConstraintBody body1 = makeConstraintBody(componentIndexBody1, &mSpringJointComponents.mI1[i]);
        const AxisConstraintBody body2 = makeConstraintBody(componentIndexBody2, &mSpringJointComponents.mI2[i]);

        mSpringJointComponents.mConstraintPart[i].warmStart(body1, body2, mSpringJointComponents.mAxisWorld[i]);
    }
}

// Solve the velocity constraint
void SolveSpringJointSystem::solveVelocityConstraint() {

    // For each joint component
    const uint32 nbJoints = mSpringJointComponents.getNbEnabledComponents();
    for (uint32 i=0; i < nbJoints; i++) {

        const Entity jointEntity = mSpringJointComponents.mJointEntities[i];
        const uint32 jointIndex = mJointComponents.getEntityIndex(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(mJointComponents.mBody1Entities[jointIndex]);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(mJointComponents.mBody2Entities[jointIndex]);

        const AxisConstraintBody body1 = makeConstraintBody(componentIndexBody1, &mSpringJointComponents.mI1[i]);
        const AxisConstraintBody body2 = makeConstraintBody(componentIndexBody2, &mSpringJointComponents.mI2[i]);

        // The spring can both push and pull: no clamping of the impulse
        mSpringJointComponents.mConstraintPart[i].solveVelocityConstraint(body1, body2, mSpringJointComponents.mAxisWorld[i],
                                                                          DECIMAL_SMALLEST, DECIMAL_LARGEST);
    }
}

// Solve the position constraint (for a rigid rod, i.e. a spring with no stiffness/damping)
void SolveSpringJointSystem::solvePositionConstraint() {

    // For each joint component
    const uint32 nbJoints = mSpringJointComponents.getNbEnabledComponents();
    for (uint32 i=0; i < nbJoints; i++) {

        const Entity jointEntity = mSpringJointComponents.mJointEntities[i];
        const uint32 jointIndex = mJointComponents.getEntityIndex(jointEntity);

        // If the error position correction technique is not the non-linear-gauss-seidel, we do
        // do not execute this method
        if (mJointComponents.mPositionCorrectionTechniques[jointIndex] != JointsPositionCorrectionTechnique::NON_LINEAR_GAUSS_SEIDEL) continue;

        // A soft spring corrects its own position error through its bias, only a rigid rod
        // needs to be corrected here
        if (mSpringJointComponents.mSpringSettings[i].isSoft()) continue;

        // Get the bodies entities
        const Entity body1Entity = mJointComponents.mBody1Entities[jointIndex];
        const Entity body2Entity = mJointComponents.mBody2Entities[jointIndex];

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        Vector3& x1 = mRigidBodyComponents.mConstrainedPositions[componentIndexBody1];
        Vector3& x2 = mRigidBodyComponents.mConstrainedPositions[componentIndexBody2];
        Quaternion& q1 = mRigidBodyComponents.mConstrainedOrientations[componentIndexBody1];
        Quaternion& q2 = mRigidBodyComponents.mConstrainedOrientations[componentIndexBody2];

        // Recompute the world inverse inertia tensors
        RigidBody::computeWorldInertiaTensorInverse(q1.getMatrix(), mRigidBodyComponents.mInverseInertiaTensorsLocal[componentIndexBody1],
                                                    mSpringJointComponents.mI1[i]);
        RigidBody::computeWorldInertiaTensorInverse(q2.getMatrix(), mRigidBodyComponents.mInverseInertiaTensorsLocal[componentIndexBody2],
                                                    mSpringJointComponents.mI2[i]);

        // Recompute the lever arms and the axis from the corrected positions
        const Vector3 r1 = q1 * (mSpringJointComponents.mLocalAnchorPointBody1[i] - mRigidBodyComponents.mCentersOfMassLocal[componentIndexBody1]);
        const Vector3 r2 = q2 * (mSpringJointComponents.mLocalAnchorPointBody2[i] - mRigidBodyComponents.mCentersOfMassLocal[componentIndexBody2]);
        const Vector3 u = x2 + r2 - x1 - r1;
        const decimal length = u.length();
        if (length <= MACHINE_EPSILON) continue;
        const Vector3 axis = u / length;

        // Constraint error
        const decimal positionError = length - mSpringJointComponents.mRestLength[i];
        if (positionError == decimal(0.0)) continue;

        // Compute the inverse of the mass matrix K=JM^-1J^t (1x1 matrix)
        const Vector3 r1CrossAxis = r1.cross(axis);
        const Vector3 r2CrossAxis = r2.cross(axis);
        const Vector3 invI1R1CrossAxis = mSpringJointComponents.mI1[i] * r1CrossAxis;
        const Vector3 invI2R2CrossAxis = mSpringJointComponents.mI2[i] * r2CrossAxis;
        const decimal inverseMassBody1 = mRigidBodyComponents.mInverseMasses[componentIndexBody1];
        const decimal inverseMassBody2 = mRigidBodyComponents.mInverseMasses[componentIndexBody2];
        const decimal inverseMassMatrix = inverseMassBody1 + inverseMassBody2 +
                                          r1CrossAxis.dot(invI1R1CrossAxis) + r2CrossAxis.dot(invI2R2CrossAxis);
        if (inverseMassMatrix <= MACHINE_EPSILON) continue;

        // Compute the Lagrange multiplier lambda
        const decimal lambda = -positionError / inverseMassMatrix;

        // Compute the pseudo velocity of body 1 (pushed along -axis)
        const Vector3 v1 = -inverseMassBody1 * mRigidBodyComponents.mLinearLockAxisFactors[componentIndexBody1] * (lambda * axis);
        const Vector3 w1 = -mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody1] * (lambda * invI1R1CrossAxis);

        // Update the body center of mass and orientation of body 1
        x1 += v1;
        q1 += Quaternion(0, w1) * q1 * decimal(0.5);
        q1.normalize();

        // Compute the pseudo velocity of body 2 (pushed along +axis)
        const Vector3 v2 = inverseMassBody2 * mRigidBodyComponents.mLinearLockAxisFactors[componentIndexBody2] * (lambda * axis);
        const Vector3 w2 = mRigidBodyComponents.mAngularLockAxisFactors[componentIndexBody2] * (lambda * invI2R2CrossAxis);

        // Update the body position/orientation of body 2
        x2 += v2;
        q2 += Quaternion(0, w2) * q2 * decimal(0.5);
        q2.normalize();
    }
}
