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
#include "SpringScene.h"
#include <nanogui/nanogui.h>
#include <GLFW/glfw3.h>
#include <cmath>
#include <sstream>
#include <iomanip>

// Namespaces
using namespace openglframework;
using namespace springscene;

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
SpringScene::SpringScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon)
      : SceneDemo(name, settings, physicsCommon, true),
        mFloor(nullptr), mBar(nullptr),
        mMass(nullptr), mSpring(nullptr), mSpringVisual(nullptr),
        mChainUpper(nullptr), mChainLower(nullptr), mChainUpperSpring(nullptr), mChainLowerSpring(nullptr),
        mChainUpperVisual(nullptr), mChainLowerVisual(nullptr),
        mPendulum(nullptr), mRod(nullptr), mRodVisual(nullptr),
        mFrequency(1.0f), mDampingRatio(0.3f), mMassKg(5.0f), mRestLength(2.0f), mRigidRod(false),
        mStatusLabel(nullptr) {

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 3.5f, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(1.6);
    resetCameraToViewAll();

    mWorldSettings.worldName = name;
}

// Destructor
SpringScene::~SpringScene() {
    destroyPhysicsWorld();
}

// Create the physics world
void SpringScene::createPhysicsWorld() {

    // Gravity vector in the physics world
    mWorldSettings.gravity = rp3d::Vector3(mEngineSettings.gravity.x, mEngineSettings.gravity.y, mEngineSettings.gravity.z);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(mWorldSettings);
    mPhysicsWorld->setEventListener(this);

    createScene();
}

// Destroy the physics world
void SpringScene::destroyPhysicsWorld() {

    if (mPhysicsWorld != nullptr) {

        for (PhysicsObject* object : mPhysicsObjects) {
            delete object;
        }
        mPhysicsObjects.clear();
        mFloor = mBar = mMass = mSpringVisual = nullptr;
        mChainUpper = mChainLower = mChainUpperVisual = mChainLowerVisual = nullptr;
        mPendulum = mRodVisual = nullptr;
        mSpring = mChainUpperSpring = mChainLowerSpring = mRod = nullptr;

        mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
        mPhysicsWorld = nullptr;
    }
}

// Reset the scene
void SpringScene::reset() {

    SceneDemo::reset();

    destroyPhysicsWorld();
    createPhysicsWorld();
}

Box* SpringScene::createBox(rp3d::BodyType type, bool isSimulated, const openglframework::Vector3& size,
                            const rp3d::Vector3& position, const rp3d::Quaternion& orientation,
                            float mass, const openglframework::Color& color) {

    Box* box = new Box(type, isSimulated, size, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    box->setTransform(rp3d::Transform(position, orientation));
    box->setColor(color);
    box->setSleepingColor(color);

    if (type == rp3d::BodyType::DYNAMIC) {
        setBoxMass(box, mass);
        // Keep everything awake so the sag readout stays live
        box->getRigidBody()->setIsAllowedToSleep(false);
    }
    box->getCollider()->getMaterial().setBounciness(0.0f);
    box->getCollider()->getMaterial().setFrictionCoefficient(0.8f);

    mPhysicsObjects.push_back(box);
    return box;
}

// Set the mass of a box body (via collider density) and refresh its mass properties
void SpringScene::setBoxMass(Box* box, float mass) {

    // Mass via density so the inertia tensor is consistent with it (setMass() alone would
    // leave the tensor at whatever the default density produced)
    const rp3d::BoxShape* shape = static_cast<const rp3d::BoxShape*>(box->getCollider()->getCollisionShape());
    const rp3d::Vector3 extents = shape->getHalfExtents() * 2.0f;
    const float volume = extents.x * extents.y * extents.z;
    box->getCollider()->getMaterial().setMassDensity(mass / volume);
    box->getRigidBody()->updateMassPropertiesFromColliders();
}

// Create all the bodies and joints
void SpringScene::createScene() {

    const rp3d::Quaternion identity = rp3d::Quaternion::identity();
    const Vector3 boxSize(BOX_SIZE, BOX_SIZE, BOX_SIZE);

    // Floor, top sitting at y = 0
    mFloor = createBox(rp3d::BodyType::STATIC, true, Vector3(20, 0.5f, 20), rp3d::Vector3(0, -0.25f, 0), identity, 0, mFloorColorDemo);

    // Static bar everything hangs from
    mBar = createBox(rp3d::BodyType::STATIC, true, Vector3(8.0f, 0.2f, 0.2f), rp3d::Vector3(0, BAR_HEIGHT + 0.1f, 0), identity, 0, mObjectColorDemo);

    // ---------- Centre: the tunable mass on a spring ---------- //
    {
        const rp3d::Vector3 anchor = springAnchor();
        const rp3d::Vector3 massPos = anchor - rp3d::Vector3(0, mRestLength, 0);
        mMass = createBox(rp3d::BodyType::DYNAMIC, true, boxSize, massPos, identity, mMassKg, mObjectColorDemo);

        // Anchor 2 at the centre of the box: the spring then applies no torque and the box
        // hangs level, which keeps the readout a pure one-dimensional check
        rp3d::SpringJointInfo info(mBar->getRigidBody(), mMass->getRigidBody(), anchor, massPos);
        info.isCollisionEnabled = false;
        mSpring = dynamic_cast<rp3d::SpringJoint*>(mPhysicsWorld->createJoint(info));
        applySettings();
    }

    // ---------- Right: two masses in series ---------- //
    {
        const rp3d::Vector3 anchor(CHAIN_X, BAR_HEIGHT, 0);
        const rp3d::Vector3 upperPos = anchor - rp3d::Vector3(0, CHAIN_REST_LENGTH, 0);
        const rp3d::Vector3 lowerPos = upperPos - rp3d::Vector3(0, CHAIN_REST_LENGTH, 0);
        mChainUpper = createBox(rp3d::BodyType::DYNAMIC, true, boxSize, upperPos, identity, CHAIN_MASS, mObjectColorDemo);
        mChainLower = createBox(rp3d::BodyType::DYNAMIC, true, boxSize, lowerPos, identity, CHAIN_MASS, mObjectColorDemo);

        const rp3d::SpringSettings spring = rp3d::SpringSettings::fromFrequencyAndDampingRatio(1.5f, 0.2f);

        rp3d::SpringJointInfo upperInfo(mBar->getRigidBody(), mChainUpper->getRigidBody(), anchor, upperPos, spring);
        upperInfo.isCollisionEnabled = false;
        mChainUpperSpring = dynamic_cast<rp3d::SpringJoint*>(mPhysicsWorld->createJoint(upperInfo));

        rp3d::SpringJointInfo lowerInfo(mChainUpper->getRigidBody(), mChainLower->getRigidBody(), upperPos, lowerPos, spring);
        lowerInfo.isCollisionEnabled = false;
        mChainLowerSpring = dynamic_cast<rp3d::SpringJoint*>(mPhysicsWorld->createJoint(lowerInfo));
    }

    // ---------- Left: rigid rod pendulum ---------- //
    {
        const rp3d::Vector3 anchor(PENDULUM_X, BAR_HEIGHT, 0);
        const float angle = PENDULUM_START_ANGLE_DEG * DEG;
        const rp3d::Vector3 bobPos = anchor + rp3d::Vector3(-std::sin(angle) * PENDULUM_LENGTH, -std::cos(angle) * PENDULUM_LENGTH, 0);
        mPendulum = createBox(rp3d::BodyType::DYNAMIC, true, boxSize, bobPos, identity, PENDULUM_MASS, mObjectColorDemo);

        // No spring settings at all: a rigid distance constraint
        rp3d::SpringJointInfo info(mBar->getRigidBody(), mPendulum->getRigidBody(), anchor, bobPos, rp3d::SpringSettings());
        info.isCollisionEnabled = false;
        mRod = dynamic_cast<rp3d::SpringJoint*>(mPhysicsWorld->createJoint(info));
    }

    // Cosmetic springs and rod: no colliders, stretched between the anchors every step
    mSpringVisual = createBox(rp3d::BodyType::STATIC, false, Vector3(0.08f, 0.08f, 1.0f), rp3d::Vector3(0, 0, 0), identity, 0, mSleepingColorDemo);
    mChainUpperVisual = createBox(rp3d::BodyType::STATIC, false, Vector3(0.08f, 0.08f, 1.0f), rp3d::Vector3(0, 0, 0), identity, 0, mSleepingColorDemo);
    mChainLowerVisual = createBox(rp3d::BodyType::STATIC, false, Vector3(0.08f, 0.08f, 1.0f), rp3d::Vector3(0, 0, 0), identity, 0, mSleepingColorDemo);
    mRodVisual = createBox(rp3d::BodyType::STATIC, false, Vector3(0.08f, 0.08f, 1.0f), rp3d::Vector3(0, 0, 0), identity, 0, mSleepingColorDemo);
    updateVisuals();
}

// Push the live settings into the centre demo's joint and body
void SpringScene::applySettings() {

    if (mSpring == nullptr || mMass == nullptr) return;

    mSpring->setSpringSettings(mRigidRod ? rp3d::SpringSettings()
                                         : rp3d::SpringSettings::fromFrequencyAndDampingRatio(mFrequency, mDampingRatio));
    mSpring->setRestLength(mRestLength);
    setBoxMass(mMass, mMassKg);
}

// Stretch a cosmetic box between two world points
void SpringScene::spanBox(Box* box, const rp3d::Vector3& from, const rp3d::Vector3& to, float thickness) {

    rp3d::Vector3 delta = to - from;
    const float span = delta.length();
    if (span < 0.001f) return;

    box->setSize(Vector3(thickness, thickness, span));
    box->setTransform(rp3d::Transform(from + delta * 0.5f, rotationWithZAlong(delta)));
}

// Re-stretch the cosmetic spring visuals between their anchors
void SpringScene::updateVisuals() {

    if (mSpringVisual && mMass) {
        spanBox(mSpringVisual, springAnchor(), mMass->getRigidBody()->getTransform().getPosition(), 0.08f);
    }
    if (mChainUpperVisual && mChainUpper) {
        spanBox(mChainUpperVisual, rp3d::Vector3(CHAIN_X, BAR_HEIGHT, 0), mChainUpper->getRigidBody()->getTransform().getPosition(), 0.08f);
    }
    if (mChainLowerVisual && mChainUpper && mChainLower) {
        spanBox(mChainLowerVisual, mChainUpper->getRigidBody()->getTransform().getPosition(),
                mChainLower->getRigidBody()->getTransform().getPosition(), 0.08f);
    }
    if (mRodVisual && mPendulum) {
        spanBox(mRodVisual, rp3d::Vector3(PENDULUM_X, BAR_HEIGHT, 0), mPendulum->getRigidBody()->getTransform().getPosition(), 0.08f);
    }
}

// Add a velocity to the centre mass
void SpringScene::kick(const rp3d::Vector3& velocity) {

    if (mMass == nullptr) return;
    rp3d::RigidBody* body = mMass->getRigidBody();
    body->setLinearVelocity(body->getLinearVelocity() + velocity);
}

// Put the centre mass at rest at the unstretched spring length (a drop test)
void SpringScene::releaseFromRestLength() {

    if (mMass == nullptr) return;
    rp3d::RigidBody* body = mMass->getRigidBody();
    body->setTransform(rp3d::Transform(springAnchor() - rp3d::Vector3(0, mRestLength, 0), rp3d::Quaternion::identity()));
    body->setLinearVelocity(rp3d::Vector3(0, 0, 0));
    body->setAngularVelocity(rp3d::Vector3(0, 0, 0));
}

// One physics step
void SpringScene::updatePhysics() {

    SceneDemo::updatePhysics();

    updateVisuals();
}

// Per frame
void SpringScene::update() {

    SceneDemo::update();

    if (mStatusLabel && mSpring && mMass) {

        const float timeStep = static_cast<float>(mEngineSettings.timeStep.count());
        const float g = std::abs(mEngineSettings.gravity.y);
        const float length = mSpring->getCurrentLength();
        const float sag = length - mSpring->getRestLength();
        const float force = mSpring->getReactionForce(timeStep).y;

        std::ostringstream out;
        out << std::fixed << std::setprecision(3)
            << "Length " << length << " m, sag " << sag << " m";
        if (!mRigidRod) {
            // Frequency mode with a static anchor at the centre of mass: k = m omega^2, so the
            // static sag m g / k = g / omega^2 does not depend on the mass at all
            const float omega = 2.0f * rp3d::PI_RP3D * mFrequency;
            out << " (static: " << (g / (omega * omega)) << " m)";
        }
        out << std::setprecision(1)
            << ", force " << force << " N (weight " << (mMassKg * g) << " N)";
        mStatusLabel->set_caption(out.str());
    }
}

bool SpringScene::keyboardEvent(int key, int scancode, int action, int mods) {

    if (SceneDemo::keyboardEvent(key, scancode, action, mods)) return true;
    if (action != GLFW_PRESS) return false;

    switch (key) {
        case GLFW_KEY_K: kick(rp3d::Vector3(0, -KICK_SPEED, 0)); return true;
        case GLFW_KEY_J: kick(rp3d::Vector3(KICK_SPEED, 0, 0)); return true;
        case GLFW_KEY_L: releaseFromRestLength(); return true;
        default: return false;
    }
}

// Scene panel controls for this scene
void SpringScene::createGuiWidgets(nanogui::Widget* parent) {

    using namespace nanogui;

    new Label(parent, "Spring Scene Demonstrates:", "sans-bold");
    auto addText = [&](const std::string& text) {
        Label* label = new Label(parent, text);
        label->set_fixed_width(230);   // wraps
        return label;
    };
    addText("Centre: one mass on a SpringJoint, tunable below. Its status line compares the measured sag and force with the analytical values.");
    addText("Right: two masses in series on two SpringJoints (1.5 Hz, ratio 0.2).");
    addText("Left: a SpringJoint with no spring settings is a rigid rod, here swinging as a pendulum.");

    new Label(parent, "Keys", "sans-bold");
    addText("K : kick the centre mass down");
    addText("J : kick the centre mass sideways");
    addText("L : release it at rest length (drop test)");

    new Label(parent, "Centre spring", "sans-bold");

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

    addSlider("Frequency (Hz)", mFrequency, 0.2f, 5.0f, 2);
    addSlider("Damping ratio", mDampingRatio, 0.0f, 2.0f, 2);
    addSlider("Mass (kg)", mMassKg, 1.0f, 50.0f, 1);
    addSlider("Rest length (m)", mRestLength, 0.5f, 4.0f, 2);

    CheckBox* rigid = new CheckBox(parent, "Rigid rod (no spring)");
    rigid->set_checked(mRigidRod);
    rigid->set_callback([this](bool checked) {
        mRigidRod = checked;
        applySettings();
    });

    Button* kickDown = new Button(parent, "Kick down (K)");
    kickDown->set_callback([this] { kick(rp3d::Vector3(0, -KICK_SPEED, 0)); });
    Button* kickSide = new Button(parent, "Kick sideways (J)");
    kickSide->set_callback([this] { kick(rp3d::Vector3(KICK_SPEED, 0, 0)); });
    Button* release = new Button(parent, "Release at rest length (L)");
    release->set_callback([this] { releaseFromRestLength(); });

    mStatusLabel = new Label(parent, "");
    mStatusLabel->set_fixed_width(230);
}
