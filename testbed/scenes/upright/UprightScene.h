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

#ifndef UPRIGHT_SCENE_H
#define UPRIGHT_SCENE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "Box.h"
#include "SceneDemo.h"

namespace nanogui { class Slider; class Label; }

namespace uprightscene {

// Constants
const float SCENE_RADIUS = 8.0f;

// Class UprightScene
/**
 * Two ship-like boxes floating in zero gravity, each with an UprightConstraint:
 *
 *  - Left: a hard cone. The box may tilt up to the cone angle and is stopped there.
 *  - Right: a soft self-righting spring (cone angle 0). The box banks under a kick and springs
 *    back upright with the configured frequency and damping.
 *
 * Both spin freely about their up axis (yaw is never constrained). Kick them with the keys and
 * watch the masts.
 */
class UprightScene : public SceneDemo {

    protected :

        // -------------------- Constants -------------------- //

        static constexpr float BODY_X = 2.5f;              // half distance between the two boxes
        static constexpr float KICK_RATE = 3.0f;           // rad/s added by a kick
        static constexpr float MAST_HEIGHT = 1.5f;

        // -------------------- Attributes -------------------- //

        Box* mHardBody;
        Box* mSoftBody;
        Box* mHardMast;   // cosmetic
        Box* mSoftMast;   // cosmetic
        rp3d::UprightConstraint* mHardConstraint;
        rp3d::UprightConstraint* mSoftConstraint;

        /// Live settings (kept across resets)
        float mConeAngleDeg;     // hard cone half angle
        float mFrequency;        // soft spring frequency (Hz)
        float mDampingRatio;     // soft spring damping ratio

        nanogui::Label* mHardLabel;
        nanogui::Label* mSoftLabel;

        /// World settings
        rp3d::PhysicsWorld::WorldSettings mWorldSettings;

        // -------------------- Methods -------------------- //

        /// Create a box body with a given mass (via collider density), position and orientation
        Box* createBox(rp3d::BodyType type, bool isSimulated, const openglframework::Vector3& size,
                       const rp3d::Vector3& position, const rp3d::Quaternion& orientation,
                       float mass, const openglframework::Color& color);

        /// Create the two bodies and their constraints
        void createScene();

        /// Push the live settings into the constraints
        void applySettings();

        /// Place the cosmetic masts along the up axes of the bodies
        void updateVisuals();

        /// Stretch a cosmetic box between two world points
        static void spanBox(Box* box, const rp3d::Vector3& from, const rp3d::Vector3& to, float thickness);

        /// Add an angular velocity (world axes) to both bodies
        void kick(const rp3d::Vector3& angularVelocity);

        /// Put both bodies back upright at rest
        void resetBodies();

        /// Angle (degrees) between the up axis of a body and world up
        static float tiltDegrees(Box* box);

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        UprightScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon);

        /// Destructor
        virtual ~UprightScene() override;

        /// Reset the scene
        virtual void reset() override;

        /// Create the physics world
        void createPhysicsWorld();

        /// Destroy the physics world
        void destroyPhysicsWorld();

        /// One physics step, then refresh the visuals
        virtual void updatePhysics() override;

        /// Per frame: refresh the status labels
        virtual void update() override;

        /// Scene panel controls
        virtual void createGuiWidgets(nanogui::Widget* parent) override;

        /// Keys: K kick roll, J kick pitch, Y kick yaw, L flip upside down, U back upright
        virtual bool keyboardEvent(int key, int scancode, int action, int mods) override;
};

}

#endif
