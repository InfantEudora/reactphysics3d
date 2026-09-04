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

#ifndef VEHICLE_SCENE_H
#define VEHICLE_SCENE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "Box.h"
#include "Sphere.h"
#include "SceneDemo.h"

namespace nanogui { class Slider; class Label; }

namespace vehiclescene {

// Constants
const float SCENE_RADIUS = 12.0f;
const int NB_WHEELS = 4;

// Class VehicleScene
/**
 * A four-wheeled chassis on a VehicleConstraint (raycast wheel suspension). The wheels are
 * rays, not bodies: the spheres and struts are cosmetic and follow what the constraint reports.
 *
 * Suspension and tire friction, no steering or drive yet: drop the car flat or tilted, push it
 * down, forward or sideways, drop it onto the ramp, and tune frequency, damping, mass and tire
 * friction live. The status lines show each wheel's suspension length and normal, longitudinal
 * and lateral forces, and the total normal force against the weight.
 */
class VehicleScene : public SceneDemo {

    protected :

        // -------------------- Constants -------------------- //

        static constexpr float CHASSIS_HALF_WIDTH = 0.8f;
        static constexpr float CHASSIS_HALF_HEIGHT = 0.25f;
        static constexpr float CHASSIS_HALF_LENGTH = 1.8f;
        static constexpr float WHEEL_RADIUS = 0.3f;
        static constexpr float WHEEL_X = 0.8f;             // half track
        static constexpr float WHEEL_Z = 1.4f;             // half wheelbase
        static constexpr float WHEEL_ANCHOR_Y = -0.2f;     // strut attachment below the chassis centre
        static constexpr float SUSPENSION_MIN = 0.2f;
        static constexpr float SUSPENSION_MAX = 0.5f;
        static constexpr float SPAWN_HEIGHT = 1.3f;        // chassis centre at spawn (wheels just off the ground)
        static constexpr float DROP_HEIGHT = 2.5f;
        static constexpr float TILT_DROP_ANGLE_DEG = 20.0f;
        static constexpr float PUSH_SPEED = 3.0f;          // m/s added by the push action
        static constexpr float RAMP_X = 6.0f;
        static constexpr float RAMP_ANGLE_DEG = 12.0f;

        // -------------------- Attributes -------------------- //

        Box* mFloor;
        Box* mRamp;
        Box* mChassis;
        rp3d::VehicleConstraint* mVehicle;
        Sphere* mWheelVisuals[NB_WHEELS];   // cosmetic
        Box* mStrutVisuals[NB_WHEELS];      // cosmetic

        /// Live settings (kept across resets)
        float mFrequency;        // Hz
        float mDampingRatio;     // 1 = critical
        float mMassKg;
        float mTireFriction;     // longitudinal and lateral friction coefficient

        nanogui::Label* mStatusLabel;
        nanogui::Label* mWheelLabels[NB_WHEELS];   // one line per wheel (nanogui labels do not wrap on newlines)

        /// World settings
        rp3d::PhysicsWorld::WorldSettings mWorldSettings;

        // -------------------- Methods -------------------- //

        /// Create a box body with a given mass (via collider density), position and orientation
        Box* createBox(rp3d::BodyType type, bool isSimulated, const openglframework::Vector3& size,
                       const rp3d::Vector3& position, const rp3d::Quaternion& orientation,
                       float mass, const openglframework::Color& color);

        /// Set the mass of a box body (via collider density) and refresh its mass properties
        static void setBoxMass(Box* box, float mass);

        /// Create all the bodies and the vehicle
        void createScene();

        /// Push the live settings into the vehicle
        void applySettings();

        /// Move the cosmetic wheels and struts to where the constraint says they are
        void updateVisuals();

        /// Stretch a cosmetic box between two world points
        static void spanBox(Box* box, const rp3d::Vector3& from, const rp3d::Vector3& to, float thickness);

        /// Add a velocity (chassis local axes) to the chassis
        void push(const rp3d::Vector3& localVelocity);

        /// Teleport the chassis to a pose at rest
        void placeChassis(const rp3d::Vector3& position, const rp3d::Quaternion& orientation);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        VehicleScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon);

        /// Destructor
        virtual ~VehicleScene() override;

        /// Reset the scene
        virtual void reset() override;

        /// Create the physics world
        void createPhysicsWorld();

        /// Destroy the physics world
        void destroyPhysicsWorld();

        /// One physics step, then refresh the visuals
        virtual void updatePhysics() override;

        /// Per frame: refresh the status label
        virtual void update() override;

        /// Scene panel controls
        virtual void createGuiWidgets(nanogui::Widget* parent) override;

        /// Keys: D drop, T tilted drop, K push down, N drop on the ramp, F push forward, S push sideways
        virtual bool keyboardEvent(int key, int scancode, int action, int mods) override;
};

}

#endif
