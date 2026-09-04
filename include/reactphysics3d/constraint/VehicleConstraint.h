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

#ifndef REACTPHYSICS3D_VEHICLE_CONSTRAINT_H
#define REACTPHYSICS3D_VEHICLE_CONSTRAINT_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/mathematics/mathematics.h>
#include <reactphysics3d/containers/Array.h>
#include <reactphysics3d/constraint/SpringSettings.h>
#include <reactphysics3d/constraint/AxisConstraintPart.h>

namespace reactphysics3d {

// Declarations
class PhysicsWorld;
class RigidBody;
class MemoryAllocator;

// Structure VehicleWheelSettings
/**
 * Describes one wheel of a VehicleConstraint: where its suspension attaches to the chassis,
 * which way it travels, how long it is and how it springs. All positions and directions are
 * in the local space of the chassis body.
 */
struct VehicleWheelSettings {

    public :

        // -------------------- Attributes -------------------- //

        /// Attachment point of the suspension on the chassis (the hard point the strut hangs from)
        Vector3 position;

        /// Direction the suspension extends in (points down for a normal car)
        Vector3 suspensionDirection;

        /// Suspension length at full compression (m), measured from position along suspensionDirection
        decimal suspensionMinLength;

        /// Suspension length at full droop (m). This is also the rest length of the spring.
        decimal suspensionMaxLength;

        /// Extra spring compression at full droop (m). The spring's natural length is
        /// suspensionMaxLength + suspensionPreloadLength, so the wheel already pushes at full
        /// droop. Note this makes touch-down a discontinuity and hence bouncier.
        decimal suspensionPreloadLength;

        /// The suspension spring. In SpringMode::FREQUENCY_AND_DAMPING_RATIO the coefficients are
        /// derived from the chassis mass and inertia as seen at this wheel, so all wheels of a
        /// car tuned to 1.5 Hz bounce at 1.5 Hz whatever the chassis. Default 1.5 Hz, ratio 0.5.
        SpringSettings suspensionSpring;

        /// Radius of the wheel (m). The tread, not the hub, touches the ground.
        decimal radius;

        /// Width of the wheel (m). Only used for rendering helpers.
        decimal width;

        /// If true, suspension forces are applied at suspensionForcePoint (fixed on the chassis)
        /// instead of at the contact point. Less accurate against dynamic ground, more stable.
        bool enableSuspensionForcePoint;

        /// Where suspension forces are applied when enableSuspensionForcePoint is set (chassis
        /// local space). A good default is the wheel centre at mid travel.
        Vector3 suspensionForcePoint;

        // -------------------- Methods -------------------- //

        /// Constructor
        VehicleWheelSettings()
            : position(0, 0, 0), suspensionDirection(0, -1, 0),
              suspensionMinLength(decimal(0.3)), suspensionMaxLength(decimal(0.5)), suspensionPreloadLength(decimal(0.0)),
              suspensionSpring(SpringSettings::fromFrequencyAndDampingRatio(decimal(1.5), decimal(0.5))),
              radius(decimal(0.3)), width(decimal(0.1)),
              enableSuspensionForcePoint(false), suspensionForcePoint(0, 0, 0) {}
};

// Class VehicleWheel
/**
 * Runtime state of one wheel of a VehicleConstraint: its settings plus what the solver found
 * this step (ground contact, suspension length, suspension impulse).
 */
class VehicleWheel {

    private :

        // -------------------- Attributes -------------------- //

        /// Settings of the wheel (may be changed at any time)
        VehicleWheelSettings mSettings;

        /// Body the wheel is touching (nullptr if in the air)
        RigidBody* mContactBody;

        /// Index of the contact body in the rigid body components (valid for the current step only)
        uint32 mContactBodyComponentIndex;

        /// True if the contact body cannot receive impulses this step (static, kinematic or sleeping)
        bool mIsContactBodyFixed;

        /// Contact point on the ground (world space)
        Vector3 mContactPoint;

        /// Contact normal (world space, pointing from the ground towards the vehicle)
        Vector3 mContactNormal;

        /// Current suspension length (m) from the attachment point along the suspension direction
        decimal mSuspensionLength;

        /// The suspension spring constraint along the contact normal
        AxisConstraintPart mSuspensionPart;

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        VehicleWheel(const VehicleWheelSettings& settings);

        /// Return the settings of the wheel
        const VehicleWheelSettings& getSettings() const;

        /// Return the settings of the wheel (writable, changes apply from the next step)
        VehicleWheelSettings& getSettings();

        /// Return true if the wheel is touching something
        bool hasContact() const;

        /// Return the body the wheel is touching (nullptr if none)
        RigidBody* getContactBody() const;

        /// Return the contact point in world space (only meaningful if hasContact())
        const Vector3& getContactPoint() const;

        /// Return the contact normal in world space (only meaningful if hasContact())
        const Vector3& getContactNormal() const;

        /// Return the current suspension length (m)
        decimal getSuspensionLength() const;

        /// Return the impulse (N.s) the suspension spring applied to the chassis this step
        decimal getSuspensionImpulse() const;

        // -------------------- Friendship -------------------- //

        friend class VehicleConstraint;
        friend class SolveVehicleSystem;
};

// Structure VehicleConstraintSettings
/**
 * Vehicle-wide settings of a VehicleConstraint. Wheels are added after creation with
 * VehicleConstraint::addWheel().
 */
struct VehicleConstraintSettings {

    public :

        // -------------------- Attributes -------------------- //

        /// Up direction of the vehicle in chassis local space
        Vector3 up;

        /// Forward direction of the vehicle in chassis local space
        Vector3 forward;

        /// Steepest surface (angle from horizontal, radians) the wheels treat as ground. Steeper
        /// surfaces hit by a wheel ray are ignored so a wall is not mistaken for the road.
        decimal maxSlopeAngle;

        /// Collision category bits the wheel rays test against (see Collider::setCollisionCategoryBits)
        unsigned short raycastCategoryMaskBits;

        // -------------------- Methods -------------------- //

        /// Constructor
        VehicleConstraintSettings()
            : up(0, 1, 0), forward(0, 0, 1), maxSlopeAngle(decimal(80.0) * PI_RP3D / decimal(180.0)),
              raycastCategoryMaskBits(0xFFFF) {}
};

// Class VehicleConstraint
/**
 * A wheeled vehicle: one chassis rigid body plus any number of wheels, each a raycast
 * suspension. Every step the solver casts a ray from each wheel's attachment point along its
 * suspension direction; where it hits the ground a spring-damper constraint (AxisConstraintPart)
 * along the contact normal pushes the chassis and the ground body apart, solved implicitly
 * together with the other constraints so it is stable for any stiffness and time step.
 *
 * The wheels themselves have no collider: the chassis is the only body, and the wheel is a
 * ray. This is the usual arcade/simulation compromise (see Jolt's VehicleConstraint, which
 * this follows): cheap, stable, and it interacts with static and dynamic ground alike.
 *
 * Create with PhysicsWorld::createVehicle(), destroy with PhysicsWorld::destroyVehicle().
 * Destroying the chassis body destroys the vehicle too.
 *
 * Current scope: suspension only. Hard stop at minimum length, tire friction, steering and
 * drive come in later steps.
 */
class VehicleConstraint {

    private :

        // -------------------- Attributes -------------------- //

        /// Reference to the physics world
        PhysicsWorld& mWorld;

        /// The chassis body
        RigidBody* mBody;

        /// Up direction of the vehicle in chassis local space
        Vector3 mUp;

        /// Forward direction of the vehicle in chassis local space
        Vector3 mForward;

        /// Cosine of the max slope angle
        decimal mCosMaxSlopeAngle;

        /// Collision category bits the wheel rays test against
        unsigned short mRaycastCategoryMaskBits;

        /// The wheels
        Array<VehicleWheel> mWheels;

        /// Index of the chassis in the rigid body components (valid for the current step only)
        uint32 mBodyComponentIndex;

        /// True if the constraint is being solved this step (chassis dynamic and awake)
        bool mIsActiveThisStep;

        // -------------------- Methods -------------------- //

        /// Constructor
        VehicleConstraint(PhysicsWorld& world, RigidBody* body, const VehicleConstraintSettings& settings,
                          MemoryAllocator& allocator);

        /// Destructor
        ~VehicleConstraint() = default;

    public :

        // -------------------- Methods -------------------- //

        /// Deleted copy-constructor
        VehicleConstraint(const VehicleConstraint& vehicle) = delete;

        /// Deleted assignment operator
        VehicleConstraint& operator=(const VehicleConstraint& vehicle) = delete;

        /// Return the chassis body
        RigidBody* getBody() const;

        /// Add a wheel and return its index
        uint32 addWheel(const VehicleWheelSettings& settings);

        /// Return the number of wheels
        uint32 getNbWheels() const;

        /// Return a wheel
        const VehicleWheel& getWheel(uint32 index) const;

        /// Return a wheel (writable, to change its settings)
        VehicleWheel& getWheel(uint32 index);

        /// Return the up direction of the vehicle in chassis local space
        const Vector3& getLocalUp() const;

        /// Return the forward direction of the vehicle in chassis local space
        const Vector3& getLocalForward() const;

        /// Return the max slope angle (radians)
        decimal getMaxSlopeAngle() const;

        /// Set the max slope angle (radians)
        void setMaxSlopeAngle(decimal maxSlopeAngle);

        /// Return the collision category bits the wheel rays test against
        unsigned short getRaycastCategoryMaskBits() const;

        /// Set the collision category bits the wheel rays test against
        void setRaycastCategoryMaskBits(unsigned short maskBits);

        /// Return the world-space centre of a wheel (attachment point + suspension length along
        /// the suspension direction), for rendering
        Vector3 getWheelCenterWorld(uint32 index) const;

        /// Return the total force (N) the suspension applied to the chassis this step, summed over
        /// all wheels along their contact normals
        decimal getTotalSuspensionForce(decimal timeStep) const;

        // -------------------- Friendship -------------------- //

        friend class PhysicsWorld;
        friend class SolveVehicleSystem;
};

// Return the settings of the wheel
RP3D_FORCE_INLINE const VehicleWheelSettings& VehicleWheel::getSettings() const {
    return mSettings;
}

// Return the settings of the wheel (writable)
RP3D_FORCE_INLINE VehicleWheelSettings& VehicleWheel::getSettings() {
    return mSettings;
}

// Return true if the wheel is touching something
RP3D_FORCE_INLINE bool VehicleWheel::hasContact() const {
    return mContactBody != nullptr;
}

// Return the body the wheel is touching
RP3D_FORCE_INLINE RigidBody* VehicleWheel::getContactBody() const {
    return mContactBody;
}

// Return the contact point in world space
RP3D_FORCE_INLINE const Vector3& VehicleWheel::getContactPoint() const {
    return mContactPoint;
}

// Return the contact normal in world space
RP3D_FORCE_INLINE const Vector3& VehicleWheel::getContactNormal() const {
    return mContactNormal;
}

// Return the current suspension length
RP3D_FORCE_INLINE decimal VehicleWheel::getSuspensionLength() const {
    return mSuspensionLength;
}

// Return the impulse the suspension spring applied this step
RP3D_FORCE_INLINE decimal VehicleWheel::getSuspensionImpulse() const {
    return mSuspensionPart.getTotalLambda();
}

// Return the chassis body
RP3D_FORCE_INLINE RigidBody* VehicleConstraint::getBody() const {
    return mBody;
}

// Return the number of wheels
RP3D_FORCE_INLINE uint32 VehicleConstraint::getNbWheels() const {
    return static_cast<uint32>(mWheels.size());
}

// Return a wheel
RP3D_FORCE_INLINE const VehicleWheel& VehicleConstraint::getWheel(uint32 index) const {
    assert(index < mWheels.size());
    return mWheels[index];
}

// Return a wheel (writable)
RP3D_FORCE_INLINE VehicleWheel& VehicleConstraint::getWheel(uint32 index) {
    assert(index < mWheels.size());
    return mWheels[index];
}

// Return the up direction of the vehicle in chassis local space
RP3D_FORCE_INLINE const Vector3& VehicleConstraint::getLocalUp() const {
    return mUp;
}

// Return the forward direction of the vehicle in chassis local space
RP3D_FORCE_INLINE const Vector3& VehicleConstraint::getLocalForward() const {
    return mForward;
}

// Return the max slope angle (radians)
RP3D_FORCE_INLINE decimal VehicleConstraint::getMaxSlopeAngle() const {
    return std::acos(mCosMaxSlopeAngle);
}

// Set the max slope angle (radians)
RP3D_FORCE_INLINE void VehicleConstraint::setMaxSlopeAngle(decimal maxSlopeAngle) {
    mCosMaxSlopeAngle = std::cos(maxSlopeAngle);
}

// Return the collision category bits the wheel rays test against
RP3D_FORCE_INLINE unsigned short VehicleConstraint::getRaycastCategoryMaskBits() const {
    return mRaycastCategoryMaskBits;
}

// Set the collision category bits the wheel rays test against
RP3D_FORCE_INLINE void VehicleConstraint::setRaycastCategoryMaskBits(unsigned short maskBits) {
    mRaycastCategoryMaskBits = maskBits;
}

}

#endif
