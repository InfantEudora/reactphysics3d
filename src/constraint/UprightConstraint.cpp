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
#include <reactphysics3d/constraint/UprightConstraint.h>
#include <reactphysics3d/body/RigidBody.h>
#include <reactphysics3d/engine/PhysicsWorld.h>

using namespace reactphysics3d;

// Constructor
UprightConstraint::UprightConstraint(PhysicsWorld& world, RigidBody* body, const UprightConstraintSettings& settings)
                  : mWorld(world), mBody(body), mSettings(settings), mCosMaxAngle(decimal(0.0)), mCurrentAngle(decimal(0.0)),
                    mRotationAxis(1, 0, 0), mSpinAxis(0, 1, 0), mBodyComponentIndex(0), mIsActiveThisStep(false) {

    assert(body != nullptr);
    mSettings.localAxis = settings.localAxis.getUnit();
    mSettings.worldAxis = settings.worldAxis.getUnit();
    setMaxAngle(settings.maxAngle);
    setSpinDamping(settings.spinDamping);
}

// Return the torque the spin damping applied to the body this step
/**
 * @param timeStep The time step of the last simulation step
 * @return The angular impulse of the damper divided by the time step, along the body axis (N.m)
 */
Vector3 UprightConstraint::getSpinDampingTorque(decimal timeStep) const {
    assert(timeStep > MACHINE_EPSILON);
    return mSpinAxis * (mSpinDampingPart.getTotalLambda() / timeStep);
}

// Return the torque applied to the body this step to right it
/**
 * @param timeStep The time step of the last simulation step
 * @return The angular impulse of the last step divided by the time step, along the righting axis (N.m)
 */
Vector3 UprightConstraint::getReactionTorque(decimal timeStep) const {
    assert(timeStep > MACHINE_EPSILON);
    return mRotationAxis * (mPart.getTotalLambda() / timeStep);
}
