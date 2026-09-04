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
#include <reactphysics3d/constraint/SpringJoint.h>
#include <reactphysics3d/systems/ConstraintSolverSystem.h>
#include <reactphysics3d/components/RigidBodyComponents.h>
#include <reactphysics3d/engine/PhysicsWorld.h>

using namespace reactphysics3d;

// Constructor
SpringJoint::SpringJoint(Entity entity, PhysicsWorld& world, const SpringJointInfo& jointInfo)
            : Joint(entity, world) {

    // Get the transforms of the two bodies
    const Transform& body1Transform = mWorld.mTransformComponents.getTransform(jointInfo.body1->getEntity());
    const Transform& body2Transform = mWorld.mTransformComponents.getTransform(jointInfo.body2->getEntity());

    Vector3 anchorPointBody1LocalSpace;
    Vector3 anchorPointBody2LocalSpace;
    Vector3 anchorPointBody1WorldSpace;
    Vector3 anchorPointBody2WorldSpace;

    if (jointInfo.isUsingLocalSpaceAnchors) {

        anchorPointBody1LocalSpace = jointInfo.anchorPointBody1LocalSpace;
        anchorPointBody2LocalSpace = jointInfo.anchorPointBody2LocalSpace;
        anchorPointBody1WorldSpace = body1Transform * anchorPointBody1LocalSpace;
        anchorPointBody2WorldSpace = body2Transform * anchorPointBody2LocalSpace;
    }
    else {

        anchorPointBody1WorldSpace = jointInfo.anchorPointBody1WorldSpace;
        anchorPointBody2WorldSpace = jointInfo.anchorPointBody2WorldSpace;
        anchorPointBody1LocalSpace = body1Transform.getInverse() * anchorPointBody1WorldSpace;
        anchorPointBody2LocalSpace = body2Transform.getInverse() * anchorPointBody2WorldSpace;
    }

    mWorld.mSpringJointsComponents.setLocalAnchorPointBody1(entity, anchorPointBody1LocalSpace);
    mWorld.mSpringJointsComponents.setLocalAnchorPointBody2(entity, anchorPointBody2LocalSpace);

    // A negative rest length means "the current distance between the anchors"
    decimal restLength = jointInfo.restLength;
    if (restLength < decimal(0.0)) {
        restLength = (anchorPointBody2WorldSpace - anchorPointBody1WorldSpace).length();
    }
    mWorld.mSpringJointsComponents.setRestLength(entity, restLength);
    mWorld.mSpringJointsComponents.setSpringSettings(entity, jointInfo.springSettings);
}

// Return the spring settings
/**
 * @return The spring settings of the joint
 */
const SpringSettings& SpringJoint::getSpringSettings() const {
    return mWorld.mSpringJointsComponents.getSpringSettings(mEntity);
}

// Set the spring settings
/**
 * @param springSettings The new spring settings (a default-constructed one makes a rigid rod)
 */
void SpringJoint::setSpringSettings(const SpringSettings& springSettings) {
    mWorld.mSpringJointsComponents.setSpringSettings(mEntity, springSettings);
    awakeBodies();
}

// Return the rest length of the spring
/**
 * @return The rest length (in meters)
 */
decimal SpringJoint::getRestLength() const {
    return mWorld.mSpringJointsComponents.getRestLength(mEntity);
}

// Set the rest length of the spring
/**
 * @param restLength The new rest length (in meters, must be >= 0)
 */
void SpringJoint::setRestLength(decimal restLength) {
    assert(restLength >= decimal(0.0));
    mWorld.mSpringJointsComponents.setRestLength(mEntity, restLength);
    awakeBodies();
}

// Return the current distance between the two anchor points
/**
 * @return The current anchor distance (in meters)
 */
decimal SpringJoint::getCurrentLength() const {

    const Entity body1Entity = mWorld.mJointsComponents.getBody1Entity(mEntity);
    const Entity body2Entity = mWorld.mJointsComponents.getBody2Entity(mEntity);

    const Transform& transformBody1 = mWorld.mTransformComponents.getTransform(body1Entity);
    const Transform& transformBody2 = mWorld.mTransformComponents.getTransform(body2Entity);

    const Vector3 anchor1World = transformBody1 * mWorld.mSpringJointsComponents.getLocalAnchorPointBody1(mEntity);
    const Vector3 anchor2World = transformBody2 * mWorld.mSpringJointsComponents.getLocalAnchorPointBody2(mEntity);

    return (anchor2World - anchor1World).length();
}

// Return the anchor point on body 1 (in local-space coordinates of body 1)
const Vector3& SpringJoint::getLocalAnchorPointBody1() const {
    return mWorld.mSpringJointsComponents.getLocalAnchorPointBody1(mEntity);
}

// Return the anchor point on body 2 (in local-space coordinates of body 2)
const Vector3& SpringJoint::getLocalAnchorPointBody2() const {
    return mWorld.mSpringJointsComponents.getLocalAnchorPointBody2(mEntity);
}

// Return the force (in Newtons) on body 2 required to satisfy the joint constraint in world-space
/**
 * @return The current force (in Newtons) applied on body 2. A positive component along the
 *         anchor-1-to-anchor-2 direction means the spring is pushing the anchors apart.
 */
Vector3 SpringJoint::getReactionForce(decimal timeStep) const {
    assert(timeStep > MACHINE_EPSILON);
    const decimal impulse = mWorld.mSpringJointsComponents.getConstraintPart(mEntity).getTotalLambda();
    return mWorld.mSpringJointsComponents.getAxisWorld(mEntity) * (impulse / timeStep);
}

// Return the torque (in Newtons * meters) on body 2 required to satisfy the joint constraint in world-space
/**
 * @return Always zero: a spring joint applies no torque of its own
 */
Vector3 SpringJoint::getReactionTorque(decimal /*timeStep*/) const {
    return Vector3(0, 0, 0);
}

// Return a string representation
std::string SpringJoint::to_string() const {
    const SpringSettings& spring = getSpringSettings();
    return "SpringJoint{ localAnchorPointBody1=" + getLocalAnchorPointBody1().to_string() +
           ", localAnchorPointBody2=" + getLocalAnchorPointBody2().to_string() +
           ", restLength=" + std::to_string(getRestLength()) +
           ", springMode=" + (spring.mode == SpringMode::FREQUENCY_AND_DAMPING_RATIO ? "FREQUENCY_AND_DAMPING_RATIO" : "STIFFNESS_AND_DAMPING") +
           ", frequencyOrStiffness=" + std::to_string(spring.frequencyOrStiffness) +
           ", damping=" + std::to_string(spring.damping) + "}";
}
