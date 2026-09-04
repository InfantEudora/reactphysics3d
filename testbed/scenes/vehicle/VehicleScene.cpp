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
#include "VehicleScene.h"
#include <nanogui/nanogui.h>
#include <GLFW/glfw3.h>
#include <cmath>
#include <sstream>
#include <iomanip>

// Namespaces
using namespace openglframework;
using namespace vehiclescene;

static const float DEG = rp3d::PI_RP3D / 180.0f;

// Collision categories: the wheel rays must only see the ground, never the cosmetic wheels and
// struts that sit exactly where the rays travel (a ray hitting its own wheel visual would read as
// a compressed suspension and launch the car)
static const unsigned short CATEGORY_GROUND = 0x0001;
static const unsigned short CATEGORY_COSMETIC = 0x0002;

// Make a body purely visual: its own category, colliding with nothing, invisible to the wheel rays
static void makeCosmetic(PhysicsObject* object) {
    rp3d::Collider* collider = object->getRigidBody()->getCollider(0);
    collider->setCollisionCategoryBits(CATEGORY_COSMETIC);
    collider->setCollideWithMaskBits(0);
}

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
VehicleScene::VehicleScene(const std::string& name, EngineSettings& settings, reactphysics3d::PhysicsCommon& physicsCommon)
      : SceneDemo(name, settings, physicsCommon, true),
        mFloor(nullptr), mRamp(nullptr), mChassis(nullptr), mVehicle(nullptr),
        mFrequency(1.5f), mDampingRatio(0.5f), mMassKg(1000.0f), mTireFriction(1.0f),
        mEngineTorque(600.0f), mBrakeTorque(1500.0f), mMaxSteerDeg(30.0f),
        mThrottle(0.0f), mSteerInput(0.0f), mBraking(false),
        mLastChassisPosition(0, SPAWN_HEIGHT, 0),
        mStatusLabel(nullptr) {

    for (int i = 0; i < NB_WHEELS; i++) {
        mWheelVisuals[i] = nullptr;
        mStrutVisuals[i] = nullptr;
        mWheelLabels[i] = nullptr;
    }

    // Compute the radius and the center of the scene
    openglframework::Vector3 center(0, 0.8f, 0);

    // Set the center of the scene
    setScenePosition(center, SCENE_RADIUS);
    setInitZoom(1.2);
    resetCameraToViewAll();

    mWorldSettings.worldName = name;
}

// Destructor
VehicleScene::~VehicleScene() {
    destroyPhysicsWorld();
}

// Create the physics world
void VehicleScene::createPhysicsWorld() {

    // Gravity vector in the physics world
    mWorldSettings.gravity = rp3d::Vector3(mEngineSettings.gravity.x, mEngineSettings.gravity.y, mEngineSettings.gravity.z);

    // Create the physics world for the physics simulation
    mPhysicsWorld = mPhysicsCommon.createPhysicsWorld(mWorldSettings);
    mPhysicsWorld->setEventListener(this);

    createScene();
}

// Destroy the physics world
void VehicleScene::destroyPhysicsWorld() {

    if (mPhysicsWorld != nullptr) {

        // The vehicle goes with its chassis body, which goes with the world
        for (PhysicsObject* object : mPhysicsObjects) {
            delete object;
        }
        mPhysicsObjects.clear();
        mFloor = mRamp = mChassis = nullptr;
        mVehicle = nullptr;
        for (int i = 0; i < NB_WHEELS; i++) {
            mWheelVisuals[i] = nullptr;
            mStrutVisuals[i] = nullptr;
        }

        mPhysicsCommon.destroyPhysicsWorld(mPhysicsWorld);
        mPhysicsWorld = nullptr;
    }
}

// Reset the scene
void VehicleScene::reset() {

    SceneDemo::reset();

    destroyPhysicsWorld();
    createPhysicsWorld();

    // Bring the camera back to the car
    const rp3d::Vector3 chassisPosition = mChassis->getRigidBody()->getTransform().getPosition();
    const rp3d::Vector3 delta = chassisPosition - mLastChassisPosition;
    mCamera.translateWorld(Vector3(delta.x, 0, delta.z));
    mCenterScene += Vector3(delta.x, 0, delta.z);
    mLastChassisPosition = chassisPosition;
}

Box* VehicleScene::createBox(rp3d::BodyType type, bool isSimulated, const openglframework::Vector3& size,
                             const rp3d::Vector3& position, const rp3d::Quaternion& orientation,
                             float mass, const openglframework::Color& color) {

    Box* box = new Box(type, isSimulated, size, mPhysicsCommon, mPhysicsWorld, mMeshFolderPath);
    box->setTransform(rp3d::Transform(position, orientation));
    box->setColor(color);
    box->setSleepingColor(color);

    if (type == rp3d::BodyType::DYNAMIC) {
        setBoxMass(box, mass);
        box->getRigidBody()->setIsAllowedToSleep(false);
    }
    box->getCollider()->getMaterial().setBounciness(0.0f);
    box->getCollider()->getMaterial().setFrictionCoefficient(0.8f);

    mPhysicsObjects.push_back(box);
    return box;
}

// Set the mass of a box body (via collider density) and refresh its mass properties
void VehicleScene::setBoxMass(Box* box, float mass) {

    const rp3d::BoxShape* shape = static_cast<const rp3d::BoxShape*>(box->getCollider()->getCollisionShape());
    const rp3d::Vector3 extents = shape->getHalfExtents() * 2.0f;
    const float volume = extents.x * extents.y * extents.z;
    box->getCollider()->getMaterial().setMassDensity(mass / volume);
    box->getRigidBody()->updateMassPropertiesFromColliders();
}

// Create all the bodies and the vehicle
void VehicleScene::createScene() {

    const rp3d::Quaternion identity = rp3d::Quaternion::identity();

    // Floor, top sitting at y = 0, large enough to drive around on
    mFloor = createBox(rp3d::BodyType::STATIC, true, Vector3(200, 0.5f, 200), rp3d::Vector3(0, -0.25f, 0), identity, 0, mFloorColorDemo);

    // Ramp: a slab tilted about the Z axis, its uphill edge towards -x
    const rp3d::Quaternion rampRotation = rp3d::Quaternion::fromEulerAngles(0, 0, RAMP_ANGLE_DEG * DEG);
    mRamp = createBox(rp3d::BodyType::STATIC, true, Vector3(6.0f, 0.3f, 6.0f), rp3d::Vector3(RAMP_X, 0.5f, 0), rampRotation, 0, mObjectColorDemo);

    // Chassis: 1.6 x 0.5 x 3.6 box, forward along +z
    mChassis = createBox(rp3d::BodyType::DYNAMIC, true,
                         Vector3(2.0f * CHASSIS_HALF_WIDTH, 2.0f * CHASSIS_HALF_HEIGHT, 2.0f * CHASSIS_HALF_LENGTH),
                         rp3d::Vector3(0, SPAWN_HEIGHT, 0), identity, mMassKg, mObjectColorDemo);
    mLastChassisPosition = rp3d::Vector3(0, SPAWN_HEIGHT, 0);

    // The vehicle: four wheels at the corners, struts hanging straight down. Wheels 0 and 1 are
    // the front (+z) left (+x) and right, 2 and 3 the rear.
    rp3d::VehicleConstraintSettings vehicleSettings;
    vehicleSettings.raycastCategoryMaskBits = CATEGORY_GROUND;
    mVehicle = mPhysicsWorld->createVehicle(mChassis->getRigidBody(), vehicleSettings);
    for (int i = 0; i < NB_WHEELS; i++) {
        rp3d::VehicleWheelSettings wheel;
        wheel.position = rp3d::Vector3((i % 2 == 0) ? WHEEL_X : -WHEEL_X, WHEEL_ANCHOR_Y, (i < 2) ? WHEEL_Z : -WHEEL_Z);
        wheel.suspensionDirection = rp3d::Vector3(0, -1, 0);
        wheel.suspensionMinLength = SUSPENSION_MIN;
        wheel.suspensionMaxLength = SUSPENSION_MAX;
        wheel.radius = WHEEL_RADIUS;
        wheel.width = WHEEL_WIDTH;
        mVehicle->addWheel(wheel);
    }
    applySettings();

    // Cosmetic wheels (flat boxes so the spin shows, no collider) and struts
    for (int i = 0; i < NB_WHEELS; i++) {
        mWheelVisuals[i] = createBox(rp3d::BodyType::STATIC, false, Vector3(WHEEL_WIDTH, 2.0f * WHEEL_RADIUS, 2.0f * WHEEL_RADIUS),
                                     rp3d::Vector3(0, 0, 0), identity, 0, mSleepingColorDemo);
        makeCosmetic(mWheelVisuals[i]);

        mStrutVisuals[i] = createBox(rp3d::BodyType::STATIC, false, Vector3(0.08f, 0.08f, 1.0f), rp3d::Vector3(0, 0, 0), identity, 0, mSleepingColorDemo);
        makeCosmetic(mStrutVisuals[i]);
    }
    updateVisuals();
}

// Push the live settings into the vehicle
void VehicleScene::applySettings() {

    if (mVehicle == nullptr || mChassis == nullptr) return;

    for (rp3d::uint32 i = 0; i < mVehicle->getNbWheels(); i++) {
        rp3d::VehicleWheelSettings& wheel = mVehicle->getWheel(i).getSettings();
        wheel.suspensionSpring = rp3d::SpringSettings::fromFrequencyAndDampingRatio(mFrequency, mDampingRatio);
        wheel.longitudinalFriction = mTireFriction;
        wheel.lateralFriction = mTireFriction;
        wheel.maxSteerAngle = mMaxSteerDeg * DEG;
    }
    setBoxMass(mChassis, mMassKg);
}

// Turn the current key inputs into wheel torques and steer angles
void VehicleScene::applyDriverInputs() {

    if (mVehicle == nullptr) return;

    for (rp3d::uint32 i = 0; i < mVehicle->getNbWheels(); i++) {
        rp3d::VehicleWheel& wheel = mVehicle->getWheel(i);
        const bool isFront = i < 2;
        wheel.setSteerAngle(isFront ? mSteerInput * mMaxSteerDeg * DEG : 0.0f);
        wheel.setDriveTorque(isFront ? 0.0f : mThrottle * mEngineTorque);
        wheel.setBrakeTorque(mBraking ? mBrakeTorque : 0.0f);
    }
}

// Stretch a cosmetic box between two world points
void VehicleScene::spanBox(Box* box, const rp3d::Vector3& from, const rp3d::Vector3& to, float thickness) {

    rp3d::Vector3 delta = to - from;
    const float span = delta.length();
    if (span < 0.001f) return;

    box->setSize(Vector3(thickness, thickness, span));
    box->setTransform(rp3d::Transform(from + delta * 0.5f, rotationWithZAlong(delta)));
}

// Move the cosmetic wheels and struts to where the constraint says they are
void VehicleScene::updateVisuals() {

    if (mVehicle == nullptr || mChassis == nullptr) return;

    const rp3d::Transform& chassisTransform = mChassis->getRigidBody()->getTransform();
    for (rp3d::uint32 i = 0; i < mVehicle->getNbWheels() && i < NB_WHEELS; i++) {

        // The wheel box is modelled along +z with +y up and its width along x, exactly the
        // convention of getWheelWorldTransform()
        const rp3d::Transform wheelTransform = mVehicle->getWheelWorldTransform(i);
        if (mWheelVisuals[i]) {
            mWheelVisuals[i]->setTransform(wheelTransform);
        }
        if (mStrutVisuals[i]) {
            const rp3d::Vector3 anchor = chassisTransform * mVehicle->getWheel(i).getSettings().position;
            spanBox(mStrutVisuals[i], anchor, wheelTransform.getPosition(), 0.08f);
        }
    }
}

// Add a velocity (chassis local axes) to the chassis
void VehicleScene::push(const rp3d::Vector3& localVelocity) {

    if (mChassis == nullptr) return;
    rp3d::RigidBody* body = mChassis->getRigidBody();
    body->setLinearVelocity(body->getLinearVelocity() + body->getTransform().getOrientation() * localVelocity);
}

// Teleport the chassis to a pose at rest
void VehicleScene::placeChassis(const rp3d::Vector3& position, const rp3d::Quaternion& orientation) {

    if (mChassis == nullptr) return;
    rp3d::RigidBody* body = mChassis->getRigidBody();
    body->setTransform(rp3d::Transform(position, orientation));
    body->setLinearVelocity(rp3d::Vector3(0, 0, 0));
    body->setAngularVelocity(rp3d::Vector3(0, 0, 0));
    for (rp3d::uint32 i = 0; mVehicle != nullptr && i < mVehicle->getNbWheels(); i++) {
        mVehicle->getWheel(i).setAngularVelocity(0.0f);
    }
}

// One physics step
void VehicleScene::updatePhysics() {

    applyDriverInputs();

    SceneDemo::updatePhysics();

    updateVisuals();
}

// Per frame
void VehicleScene::update() {

    SceneDemo::update();

    // Follow the car horizontally
    if (mChassis) {
        const rp3d::Vector3 chassisPosition = mChassis->getRigidBody()->getTransform().getPosition();
        const rp3d::Vector3 delta = chassisPosition - mLastChassisPosition;
        mCamera.translateWorld(Vector3(delta.x, 0, delta.z));
        mCenterScene += Vector3(delta.x, 0, delta.z);
        mLastChassisPosition = chassisPosition;
    }

    if (mStatusLabel && mVehicle && mChassis) {

        const float timeStep = static_cast<float>(mEngineSettings.timeStep.count());
        const float g = std::abs(mEngineSettings.gravity.y);
        const rp3d::Vector3 velocity = mChassis->getRigidBody()->getLinearVelocity();
        const float speed = velocity.dot(mChassis->getRigidBody()->getTransform().getOrientation() * rp3d::Vector3(0, 0, 1));

        std::ostringstream out;
        out << std::fixed << std::setprecision(1)
            << "Speed " << (speed * 3.6f) << " km/h, N " << std::setprecision(0) << mVehicle->getTotalSuspensionForce(timeStep) << " / " << (mMassKg * g);
        mStatusLabel->set_caption(out.str());

        for (rp3d::uint32 i = 0; i < mVehicle->getNbWheels() && i < NB_WHEELS; i++) {
            if (mWheelLabels[i] == nullptr) continue;
            const rp3d::VehicleWheel& wheel = mVehicle->getWheel(i);
            std::ostringstream line;
            line << (i < 2 ? "Front " : "Rear ") << (i % 2 == 0 ? "left" : "right") << ": ";
            if (wheel.hasContact()) {
                line << std::fixed << std::setprecision(3) << wheel.getSuspensionLength() << " m"
                     << (wheel.hasHitHardStop() ? " (stop)" : "") << ", "
                     << std::setprecision(0) << "N " << (wheel.getNormalImpulse() / timeStep)
                     << ", F " << (wheel.getLongitudinalImpulse() / timeStep)
                     << ", S " << (wheel.getLateralImpulse() / timeStep)
                     << std::setprecision(1) << ", v " << (wheel.getAngularVelocity() * wheel.getSettings().radius);
            }
            else {
                line << "in the air, v " << std::fixed << std::setprecision(1) << (wheel.getAngularVelocity() * wheel.getSettings().radius);
            }
            mWheelLabels[i]->set_caption(line.str());
        }
    }
}

bool VehicleScene::keyboardEvent(int key, int scancode, int action, int mods) {

    if (SceneDemo::keyboardEvent(key, scancode, action, mods)) return true;
    if (action == GLFW_REPEAT) return false;

    // Held keys: driving
    const bool pressed = (action == GLFW_PRESS);
    switch (key) {
        case GLFW_KEY_UP:    mThrottle = pressed ? 1.0f : (mThrottle > 0.0f ? 0.0f : mThrottle); return true;
        case GLFW_KEY_DOWN:  mThrottle = pressed ? -1.0f : (mThrottle < 0.0f ? 0.0f : mThrottle); return true;
        case GLFW_KEY_LEFT:  mSteerInput = pressed ? 1.0f : (mSteerInput > 0.0f ? 0.0f : mSteerInput); return true;
        case GLFW_KEY_RIGHT: mSteerInput = pressed ? -1.0f : (mSteerInput < 0.0f ? 0.0f : mSteerInput); return true;
        case GLFW_KEY_SPACE: mBraking = pressed; return true;
        default: break;
    }

    if (!pressed) return false;

    // One-shot keys
    switch (key) {
        case GLFW_KEY_D:
            placeChassis(rp3d::Vector3(mLastChassisPosition.x, DROP_HEIGHT, mLastChassisPosition.z), rp3d::Quaternion::identity());
            return true;
        case GLFW_KEY_T:
            placeChassis(rp3d::Vector3(mLastChassisPosition.x, DROP_HEIGHT, mLastChassisPosition.z), rp3d::Quaternion::fromEulerAngles(0, 0, TILT_DROP_ANGLE_DEG * DEG));
            return true;
        case GLFW_KEY_N:
            placeChassis(rp3d::Vector3(RAMP_X, DROP_HEIGHT + 1.0f, 0), rp3d::Quaternion::fromEulerAngles(0, 0, RAMP_ANGLE_DEG * DEG));
            return true;
        case GLFW_KEY_K:
            push(rp3d::Vector3(0, -PUSH_SPEED, 0));
            return true;
        case GLFW_KEY_F:
            push(rp3d::Vector3(0, 0, PUSH_SPEED));
            return true;
        case GLFW_KEY_S:
            push(rp3d::Vector3(PUSH_SPEED, 0, 0));
            return true;
        default:
            return false;
    }
}

// Scene panel controls for this scene
void VehicleScene::createGuiWidgets(nanogui::Widget* parent) {

    using namespace nanogui;

    new Label(parent, "Vehicle Scene Demonstrates:", "sans-bold");
    auto addText = [&](const std::string& text) {
        Label* label = new Label(parent, text);
        label->set_fixed_width(230);   // wraps
        return label;
    };
    addText("A chassis on a VehicleConstraint: four raycast wheels, each a spring-damper along its contact normal with a hard stop, plus tire friction along and across the rolling direction. Rear wheel drive, front wheel steering. The wheel boxes and struts are cosmetic.");
    addText("Per wheel: suspension length, normal (N), forward (F) and sideways (S) force in newtons, wheel surface speed v in m/s.");

    new Label(parent, "Keys", "sans-bold");
    addText("Up / Down : throttle forward / reverse");
    addText("Left / Right : steer");
    addText("Space : brake");
    addText("D : drop the car flat, T : rolled 20 degrees, N : on the ramp");
    addText("K : push it down, F : forward, S : sideways");

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

    addSlider("Frequency (Hz)", mFrequency, 0.5f, 4.0f, 2);
    addSlider("Damping ratio", mDampingRatio, 0.0f, 2.0f, 2);
    addSlider("Chassis mass (kg)", mMassKg, 200.0f, 3000.0f, 0);
    addSlider("Tire friction", mTireFriction, 0.0f, 2.0f, 2);
    addSlider("Engine torque per wheel (Nm)", mEngineTorque, 0.0f, 3000.0f, 0);
    addSlider("Brake torque per wheel (Nm)", mBrakeTorque, 0.0f, 5000.0f, 0);
    addSlider("Steering lock (deg)", mMaxSteerDeg, 5.0f, 45.0f, 0);

    Button* drop = new Button(parent, "Drop flat (D)");
    drop->set_callback([this] { placeChassis(rp3d::Vector3(mLastChassisPosition.x, DROP_HEIGHT, mLastChassisPosition.z), rp3d::Quaternion::identity()); });
    Button* tilt = new Button(parent, "Drop tilted (T)");
    tilt->set_callback([this] { placeChassis(rp3d::Vector3(mLastChassisPosition.x, DROP_HEIGHT, mLastChassisPosition.z), rp3d::Quaternion::fromEulerAngles(0, 0, TILT_DROP_ANGLE_DEG * DEG)); });
    Button* ramp = new Button(parent, "Drop on ramp (N)");
    ramp->set_callback([this] { placeChassis(rp3d::Vector3(RAMP_X, DROP_HEIGHT + 1.0f, 0), rp3d::Quaternion::fromEulerAngles(0, 0, RAMP_ANGLE_DEG * DEG)); });

    mStatusLabel = new Label(parent, "Speed, suspension force vs weight");
    mStatusLabel->set_fixed_width(230);
    for (int i = 0; i < NB_WHEELS; i++) {
        // Non-empty from the start: nanogui lays an empty label out with zero height and does
        // not grow it when the caption changes later
        mWheelLabels[i] = new Label(parent, std::string(i < 2 ? "Front " : "Rear ") + (i % 2 == 0 ? "left" : "right") + ": -");
        mWheelLabels[i]->set_fixed_width(230);
    }
}
