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

#ifndef SPRING_SCENE_H
#define SPRING_SCENE_H

// Libraries
#include "openglframework.h"
#include <reactphysics3d/reactphysics3d.h>
#include "Box.h"
#include "SceneDemo.h"

namespace nanogui { class Slider; class Label; class CheckBox; }

namespace springscene {

// Constants
const float SCENE_RADIUS = 10.0f;

// Class SpringScene
/**
 * Three SpringJoint demos hanging from one static bar:
 *
 *  - Centre: a single mass on a spring, tunable live from the Scene panel (frequency, damping
 *    ratio, mass, rest length, or switch it to a rigid rod). The status line compares the
 *    measured sag and reaction force with the analytical values, so this is the place to check
 *    that the solver does what the settings say.
 *  - Right: two masses in series, each on its own spring - the effective-mass behaviour of a
 *    spring between two dynamic bodies.
 *  - Left: a rigid rod (SpringJoint with no spring) swinging as a pendulum.
 */
class SpringScene : public SceneDemo {

    protected :

        // -------------------- Constants -------------------- //

        static constexpr float BAR_HEIGHT = 6.0f;          // y of the anchor bar
        static constexpr float BOX_SIZE = 0.5f;            // edge of the hanging boxes
        static constexpr float CHAIN_X = 3.0f;             // x of the two-mass chain
        static constexpr float CHAIN_REST_LENGTH = 1.5f;
        static constexpr float CHAIN_MASS = 3.0f;
        static constexpr float PENDULUM_X = -3.0f;         // x of the rigid-rod pendulum anchor
        static constexpr float PENDULUM_LENGTH = 2.5f;
        static constexpr float PENDULUM_MASS = 4.0f;
        static constexpr float PENDULUM_START_ANGLE_DEG = 60.0f;
        static constexpr float KICK_SPEED = 2.0f;          // m/s added by the kick buttons/keys

        // -------------------- Attributes -------------------- //

        Box* mFloor;
        Box* mBar;

        /// Centre demo: the tunable mass
        Box* mMass;
        rp3d::SpringJoint* mSpring;
        Box* mSpringVisual;   // cosmetic

        /// Right demo: two masses in series
        Box* mChainUpper;
        Box* mChainLower;
        rp3d::SpringJoint* mChainUpperSpring;
        rp3d::SpringJoint* mChainLowerSpring;
        Box* mChainUpperVisual;   // cosmetic
        Box* mChainLowerVisual;   // cosmetic

        /// Left demo: rigid rod pendulum
        Box* mPendulum;
        rp3d::SpringJoint* mRod;
        Box* mRodVisual;   // cosmetic

        /// Live settings of the centre demo (kept across resets)
        float mFrequency;        // Hz
        float mDampingRatio;     // 1 = critical
        float mMassKg;
        float mRestLength;       // m
        bool mRigidRod;          // true: no spring at all, the joint is a rigid rod

        nanogui::Label* mStatusLabel;

        /// World settings
        rp3d::PhysicsWorld::WorldSettings mWorldSettings;

        // -------------------- Methods -------------------- //

        /// Create a box body with a given mass (via collider density), position and orientation
        Box* createBox(rp3d::BodyType type, bool isSimulated, const openglframework::Vector3& size,
                       const rp3d::Vector3& position, const rp3d::Quaternion& orientation,
                       float mass, const openglframework::Color& color);

        /// Set the mass of a box body (via collider density) and refresh its mass properties
        static void setBoxMass(Box* box, float mass);

        /// Create all the bodies and joints
        void createScene();

        /// Push the live settings into the centre demo's joint and body
        void applySettings();

        /// Re-stretch the cosmetic spring visuals between their anchors
        void updateVisuals();

        /// Stretch a cosmetic box between two world points
        static void spanBox(Box* box, const rp3d::Vector3& from, const rp3d::Vector3& to, float thickness);

        /// Add a velocity to the centre mass
        void kick(const rp3d::Vector3& velocity);

        /// Put the centre mass at rest at the unstretched spring length (a drop test)
        void releaseFromRestLength();

        /// World-space anchor of the centre demo on the bar
        rp3d::Vector3 springAnchor() const { return rp3d::Vector3(0, BAR_HEIGHT, 0); }

    public:

        // -------------------- Methods -------------------- //

        /// Constructor
        SpringScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon);

        /// Destructor
        virtual ~SpringScene() override;

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

        /// Keys: K kick down, J kick sideways, L release from rest length
        virtual bool keyboardEvent(int key, int scancode, int action, int mods) override;
};

}

#endif
