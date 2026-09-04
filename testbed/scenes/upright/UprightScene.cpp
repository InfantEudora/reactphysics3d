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
#include "UprightScene.h"
#include <nanogui/nanogui.h>
#include <GLFW/glfw3.h>
#include <cmath>
#include <sstream>
#include <iomanip>

// Namespaces
using namespace openglframework;
using namespace uprightscene;

static const float DEG = rp3d::PI_RP3D / 180.0f;

// Rotation that points a box's local +Z along `dir`
static rp3d::Quaternion rotationWithZAlong(const rp3d::Vector3& dir) {
    rp3d::Vector3 z = dir.getUnit();
    rp3d::Vector3 x = z.getOneUnitOrthogonalVector();
    rp3d::Vector3 y = z.cross(x);
    // Matrix3x3 is row-major; the columns are the rotated basis vectors
    return rp3d::Quaternion(rp3d::Matrix3x3(x.x, y.x, z.x,
                                            x.y, y.y, z.y,
                                            x.z, y.z, z.z));
}

// Constructor
UprightScene::UprightScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon)
      : SceneDemo(name, settings, physicsCommon, true),
        mHardBody(nullptr), mSoftBody(nullptr), mHardMast(nullptr), mSoftMast(nullptr),
        mHardConstraint(nullptr), mSoftConstraint(nullptr),
        mConeAngleDeg(30.0f), mFrequency(1.0f), mDampingRatio(0.3f), mSpinDamping(0.0f),
        mHardLabel(nullptr), mSoftLabel(nullptr) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 0.5f, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(1.4);
    resetCameraToViewAll();

    mWorldSettings.worldName = name;
}

// Destructor
UprightScene::~UprightScene() {
    destroyPhysicsWorld();
}

// Create the physics world
void UprightScene::createPhysicsWorld() {

    // Gravity vector in the physics world (the bodies below ignore it anyway: this is space)
    mWorldSettings.gravity = rp3d::Vector3(mEngineSettings.gravity.x, mEngineSettings.gravity.y, mEngineSettings.gravity.z);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(mWorldSettings);
    mPhysicsWorld->setEventListener(this);

    createScene();
}

// Destroy the physics world
void UprightScene::destroyPhysicsWorld() {

    if (mPhysicsWorld != nullptr) {

        for (PhysicsObject* object : mPhysicsObjects) {
            delete object;
        }
        mPhysicsObjects.clear();
        mHardBody = mSoftBody = mHardMast = mSoftMast = nullptr;
        mHardConstraint = mSoftConstraint = nullptr;

        mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
        mPhysicsWorld = nullptr;
    }
}

// Reset the scene
void UprightScene::reset() {

    SceneDemo::reset();

    destroyPhysicsWorld();
    createPhysicsWorld();
}

Box* UprightScene::createBox(rp3d::BodyType type, bool isSimulated, const openglframework::Vector3& size,
                             const rp3d::Vector3& position, const rp3d::Quaternion& orientation,
                             float mass, const openglframework::Color& color) {

    Box* box = new Box(type, isSimulated, size, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    box->setTransform(rp3d::Transform(position, orientation));
    box->setColor(color);
    box->setSleepingColor(color);

    if (type == rp3d::BodyType::DYNAMIC) {
        // Mass via density so the inertia tensor is consistent with it
        const float volume = size.x * size.y * size.z;
        box->getCollider()->getMaterial().setMassDensity(mass / volume);
        box->getRigidBody()->updateMassPropertiesFromColliders();
        box->getRigidBody()->setIsAllowedToSleep(false);
        // Space: no gravity on these bodies whatever the world says
        box->getRigidBody()->enableGravity(false);
    }
    // Nothing here collides with anything: the two ships never meet and the masts are visual
    box->getCollider()->setCollideWithMaskBits(0);

    mPhysicsObjects.push_back(box);
    return box;
}

// Create the two bodies and their constraints
void UprightScene::createScene() {

    const rp3d::Quaternion identity = rp3d::Quaternion::identity();
    const Vector3 shipSize(1.0f, 0.4f, 1.6f);

    // Both start rolled 40 degrees, so the difference shows at once: the hard cone pushes its
    // ship back to the cone edge, the soft spring rights its ship completely
    const rp3d::Quaternion rolled = rp3d::Quaternion::fromEulerAngles(0, 0, 40.0f * DEG);

    // Left: hard cone
    mHardBody = createBox(rp3d::BodyType::DYNAMIC, true, shipSize, rp3d::Vector3(-BODY_X, 0, 0), rolled, 10.0f, mObjectColorDemo);
    {
        rp3d::UprightConstraintSettings settings;
        settings.maxAngle = mConeAngleDeg * DEG;
        settings.spring = rp3d::SpringSettings();   // hard
        mHardConstraint = mPhysicsWorld->createUprightConstraint(mHardBody->getRigidBody(), settings);
    }

    // Right: soft self-righting spring
    mSoftBody = createBox(rp3d::BodyType::DYNAMIC, true, shipSize, rp3d::Vector3(BODY_X, 0, 0), rolled, 10.0f, mObjectColorDemo);
    {
        rp3d::UprightConstraintSettings settings;
        settings.maxAngle = 0.0f;
        settings.spring = rp3d::SpringSettings::fromFrequencyAndDampingRatio(mFrequency, mDampingRatio);
        mSoftConstraint = mPhysicsWorld->createUprightConstraint(mSoftBody->getRigidBody(), settings);
    }

    // Cosmetic masts along the up axes
    mHardMast = createBox(rp3d::BodyType::STATIC, false, Vector3(0.08f, 0.08f, 1.0f), rp3d::Vector3(0, 0, 0), identity, 0, mSleepingColorDemo);
    mSoftMast = createBox(rp3d::BodyType::STATIC, false, Vector3(0.08f, 0.08f, 1.0f), rp3d::Vector3(0, 0, 0), identity, 0, mSleepingColorDemo);
    updateVisuals();
}

// Push the live settings into the constraints
void UprightScene::applySettings() {

    if (mHardConstraint) {
        mHardConstraint->setMaxAngle(mConeAngleDeg * DEG);
        mHardConstraint->setSpinDamping(mSpinDamping);
    }
    if (mSoftConstraint) {
        mSoftConstraint->setSpringSettings(rp3d::SpringSettings::fromFrequencyAndDampingRatio(mFrequency, mDampingRatio));
        mSoftConstraint->setSpinDamping(mSpinDamping);
    }
}

// Stretch a cosmetic box between two world points
void UprightScene::spanBox(Box* box, const rp3d::Vector3& from, const rp3d::Vector3& to, float thickness) {

    rp3d::Vector3 delta = to - from;
    const float span = delta.length();
    if (span < 0.001f) return;

    box->setSize(Vector3(thickness, thickness, span));
    box->setTransform(rp3d::Transform(from + delta * 0.5f, rotationWithZAlong(delta)));
}

// Place the cosmetic masts along the up axes of the bodies
void UprightScene::updateVisuals() {

    auto placeMast = [](Box* mast, Box* body) {
        if (mast == nullptr || body == nullptr) return;
        const rp3d::Transform& transform = body->getRigidBody()->getTransform();
        const rp3d::Vector3 up = transform.getOrientation() * rp3d::Vector3(0, 1, 0);
        spanBox(mast, transform.getPosition(), transform.getPosition() + up * MAST_HEIGHT, 0.08f);
    };
    placeMast(mHardMast, mHardBody);
    placeMast(mSoftMast, mSoftBody);
}

// Add an angular velocity (world axes) to both bodies
void UprightScene::kick(const rp3d::Vector3& angularVelocity) {

    for (Box* box : { mHardBody, mSoftBody }) {
        if (box == nullptr) continue;
        rp3d::RigidBody* body = box->getRigidBody();
        body->setAngularVelocity(body->getAngularVelocity() + angularVelocity);
    }
}

// Put both bodies back upright at rest
void UprightScene::resetBodies() {

    if (mHardBody) {
        mHardBody->getRigidBody()->setTransform(rp3d::Transform(rp3d::Vector3(-BODY_X, 0, 0), rp3d::Quaternion::identity()));
        mHardBody->getRigidBody()->setAngularVelocity(rp3d::Vector3(0, 0, 0));
        mHardBody->getRigidBody()->setLinearVelocity(rp3d::Vector3(0, 0, 0));
    }
    if (mSoftBody) {
        mSoftBody->getRigidBody()->setTransform(rp3d::Transform(rp3d::Vector3(BODY_X, 0, 0), rp3d::Quaternion::identity()));
        mSoftBody->getRigidBody()->setAngularVelocity(rp3d::Vector3(0, 0, 0));
        mSoftBody->getRigidBody()->setLinearVelocity(rp3d::Vector3(0, 0, 0));
    }
}

// Angle (degrees) between the up axis of a body and world up
float UprightScene::tiltDegrees(Box* box) {

    const rp3d::Vector3 up = box->getRigidBody()->getTransform().getOrientation() * rp3d::Vector3(0, 1, 0);
    return std::acos(std::max(-1.0f, std::min(1.0f, up.y))) / DEG;
}

// One physics step
void UprightScene::updatePhysics() {

    SceneDemo::updatePhysics();

    updateVisuals();
}

// Per frame
void UprightScene::update() {

    SceneDemo::update();

    const float timeStep = static_cast<float>(mEngineSettings.timeStep.count());

    if (mHardLabel && mHardBody && mHardConstraint) {
        std::ostringstream out;
        out << std::fixed << std::setprecision(1)
            << "Hard: tilt " << tiltDegrees(mHardBody) << " deg, yaw " << mHardBody->getRigidBody()->getAngularVelocity().y
            << " rad/s, " << std::setprecision(0) << mHardConstraint->getReactionTorque(timeStep).length() << " Nm";
        mHardLabel->set_caption(out.str());
    }
    if (mSoftLabel && mSoftBody && mSoftConstraint) {
        std::ostringstream out;
        out << std::fixed << std::setprecision(1)
            << "Soft: tilt " << tiltDegrees(mSoftBody) << " deg, yaw " << mSoftBody->getRigidBody()->getAngularVelocity().y
            << " rad/s, " << std::setprecision(0) << mSoftConstraint->getReactionTorque(timeStep).length() << " Nm";
        mSoftLabel->set_caption(out.str());
    }
}

bool UprightScene::keyboardEvent(int key, int scancode, int action, int mods) {

    if (SceneDemo::keyboardEvent(key, scancode, action, mods)) return true;
    if (action != GLFW_PRESS) return false;

    switch (key) {
        case GLFW_KEY_K: kick(rp3d::Vector3(0, 0, KICK_RATE)); return true;
        case GLFW_KEY_J: kick(rp3d::Vector3(KICK_RATE, 0, 0)); return true;
        case GLFW_KEY_Y: kick(rp3d::Vector3(0, KICK_RATE, 0)); return true;
        case GLFW_KEY_L:
            for (Box* box : { mHardBody, mSoftBody }) {
                if (box == nullptr) continue;
                rp3d::RigidBody* body = box->getRigidBody();
                body->setTransform(rp3d::Transform(body->getTransform().getPosition(), rp3d::Quaternion::fromEulerAngles(0, 0, 170.0f * DEG)));
                body->setAngularVelocity(rp3d::Vector3(0, 0, 0));
            }
            return true;
        case GLFW_KEY_U: resetBodies(); return true;
        default: return false;
    }
}

// Scene panel controls for this scene
void UprightScene::createGuiWidgets(nanogui::Widget* parent) {

    using namespace nanogui;

    new Label(parent, "Upright Scene Demonstrates:", "sans-bold");
    auto addText = [&](const std::string& text) {
        Label* label = new Label(parent, text);
        label->set_fixed_width(230);   // wraps
        return label;
    };
    addText("Two ships in zero gravity, each on an UprightConstraint. Only the tilt of the mast is constrained; yaw is free unless the spin damping slider is up, which slows the spin about the mast at that rate.");
    addText("Left: a hard cone. It may tilt up to the cone angle and is stopped there.");
    addText("Right: a soft spring with a zero cone. It banks under a kick and springs back upright.");

    new Label(parent, "Keys", "sans-bold");
    addText("K : kick roll, J : kick pitch, Y : kick yaw");
    addText("L : flip both nearly upside down");
    addText("U : back upright at rest");

    new Label(parent, "Tuning", "sans-bold");

    auto addSlider = [&](const std::string& title, float& value, float minValue, float maxValue, int precision) -> Slider* {
        auto caption = [title, precision](float v) {
            std::ostringstream out;
            out << title << " : " << std::fixed << std::setprecision(precision) << v;
            return out.str();
        };
        Label* label = new Label(parent, caption(value));
        Slider* slider = new Slider(parent);
        slider->set_range(std::make_pair(minValue, maxValue));
        slider->set_value(value);
        slider->set_fixed_width(230);
        slider->set_callback([this, &value, label, caption](float v) {
            value = v;
            label->set_caption(caption(v));
            applySettings();
        });
        return slider;
    };

    addSlider("Hard cone angle (deg)", mConeAngleDeg, 0.0f, 90.0f, 0);
    addSlider("Soft frequency (Hz)", mFrequency, 0.2f, 4.0f, 2);
    addSlider("Soft damping ratio", mDampingRatio, 0.0f, 2.0f, 2);
    addSlider("Spin damping (1/s), both", mSpinDamping, 0.0f, 3.0f, 2);

    Button* roll = new Button(parent, "Kick roll (K)");
    roll->set_callback([this] { kick(rp3d::Vector3(0, 0, KICK_RATE)); });
    Button* yaw = new Button(parent, "Kick yaw (Y)");
    yaw->set_callback([this] { kick(rp3d::Vector3(0, KICK_RATE, 0)); });
    Button* flip = new Button(parent, "Flip (L)");
    flip->set_callback([this] { keyboardEvent(GLFW_KEY_L, 0, GLFW_PRESS, 0); });

    mHardLabel = new Label(parent, "Hard cone: -");
    mHardLabel->set_fixed_width(230);
    mSoftLabel = new Label(parent, "Soft spring: -");
    mSoftLabel->set_fixed_width(230);
}
