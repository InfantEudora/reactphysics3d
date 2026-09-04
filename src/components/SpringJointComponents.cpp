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
#include <reactphysics3d/components/SpringJointComponents.h>
#include <reactphysics3d/engine/EntityManager.h>
#include <reactphysics3d/mathematics/Matrix3x3.h>
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
SpringJointComponents::SpringJointComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(SpringJoint*) + sizeof(Vector3) +
                                sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) +
                                sizeof(Matrix3x3) + sizeof(Matrix3x3) + sizeof(decimal) + sizeof(decimal) +
                                sizeof(SpringSettings) + sizeof(AxisConstraintPart), 13 * GLOBAL_ALIGNMENT) {

}

// Allocate memory for a given number of components
void SpringJointComponents::allocate(uint32 nbComponentsToAllocate) {

    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Make sure capacity is an integral multiple of alignment
    nbComponentsToAllocate = std::ceil(nbComponentsToAllocate / float(GLOBAL_ALIGNMENT)) * GLOBAL_ALIGNMENT;

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * mComponentDataSize + mAlignmentMarginSize;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);
    assert(reinterpret_cast<uintptr_t>(newBuffer) % GLOBAL_ALIGNMENT == 0);

    // New pointers to components data
    Entity* newJointEntities = static_cast<Entity*>(newBuffer);
    SpringJoint** newJoints = reinterpret_cast<SpringJoint**>(MemoryAllocator::alignAddress(newJointEntities + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newJoints) % GLOBAL_ALIGNMENT == 0);
    Vector3* newLocalAnchorPointBody1 = reinterpret_cast<Vector3*>(MemoryAllocator::alignAddress(newJoints + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newLocalAnchorPointBody1) % GLOBAL_ALIGNMENT == 0);
    Vector3* newLocalAnchorPointBody2 = reinterpret_cast<Vector3*>(MemoryAllocator::alignAddress(newLocalAnchorPointBody1 + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newLocalAnchorPointBody2) % GLOBAL_ALIGNMENT == 0);
    Vector3* newR1World = reinterpret_cast<Vector3*>(MemoryAllocator::alignAddress(newLocalAnchorPointBody2 + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newR1World) % GLOBAL_ALIGNMENT == 0);
    Vector3* newR2World = reinterpret_cast<Vector3*>(MemoryAllocator::alignAddress(newR1World + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newR2World) % GLOBAL_ALIGNMENT == 0);
    Vector3* newAxisWorld = reinterpret_cast<Vector3*>(MemoryAllocator::alignAddress(newR2World + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newAxisWorld) % GLOBAL_ALIGNMENT == 0);
    Matrix3x3* newI1 = reinterpret_cast<Matrix3x3*>(MemoryAllocator::alignAddress(newAxisWorld + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newI1) % GLOBAL_ALIGNMENT == 0);
    Matrix3x3* newI2 = reinterpret_cast<Matrix3x3*>(MemoryAllocator::alignAddress(newI1 + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newI2) % GLOBAL_ALIGNMENT == 0);
    decimal* newRestLength = reinterpret_cast<decimal*>(MemoryAllocator::alignAddress(newI2 + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newRestLength) % GLOBAL_ALIGNMENT == 0);
    decimal* newCurrentLength = reinterpret_cast<decimal*>(MemoryAllocator::alignAddress(newRestLength + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newCurrentLength) % GLOBAL_ALIGNMENT == 0);
    SpringSettings* newSpringSettings = reinterpret_cast<SpringSettings*>(MemoryAllocator::alignAddress(newCurrentLength + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newSpringSettings) % GLOBAL_ALIGNMENT == 0);
    AxisConstraintPart* newConstraintPart = reinterpret_cast<AxisConstraintPart*>(MemoryAllocator::alignAddress(newSpringSettings + nbComponentsToAllocate, GLOBAL_ALIGNMENT));
    assert(reinterpret_cast<uintptr_t>(newConstraintPart) % GLOBAL_ALIGNMENT == 0);
    assert(reinterpret_cast<uintptr_t>(newConstraintPart + nbComponentsToAllocate) <= reinterpret_cast<uintptr_t>(newBuffer) + totalSizeBytes);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newJointEntities, mJointEntities, mNbComponents * sizeof(Entity));
        memcpy(newJoints, mJoints, mNbComponents * sizeof(SpringJoint*));
        memcpy(newLocalAnchorPointBody1, mLocalAnchorPointBody1, mNbComponents * sizeof(Vector3));
        memcpy(newLocalAnchorPointBody2, mLocalAnchorPointBody2, mNbComponents * sizeof(Vector3));
        memcpy(newR1World, mR1World, mNbComponents * sizeof(Vector3));
        memcpy(newR2World, mR2World, mNbComponents * sizeof(Vector3));
        memcpy(newAxisWorld, mAxisWorld, mNbComponents * sizeof(Vector3));
        memcpy(newI1, mI1, mNbComponents * sizeof(Matrix3x3));
        memcpy(newI2, mI2, mNbComponents * sizeof(Matrix3x3));
        memcpy(newRestLength, mRestLength, mNbComponents * sizeof(decimal));
        memcpy(newCurrentLength, mCurrentLength, mNbComponents * sizeof(decimal));
        memcpy(newSpringSettings, mSpringSettings, mNbComponents * sizeof(SpringSettings));
        memcpy(newConstraintPart, mConstraintPart, mNbComponents * sizeof(AxisConstraintPart));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mJointEntities = newJointEntities;
    mJoints = newJoints;
    mNbAllocatedComponents = nbComponentsToAllocate;
    mLocalAnchorPointBody1 = newLocalAnchorPointBody1;
    mLocalAnchorPointBody2 = newLocalAnchorPointBody2;
    mR1World = newR1World;
    mR2World = newR2World;
    mAxisWorld = newAxisWorld;
    mI1 = newI1;
    mI2 = newI2;
    mRestLength = newRestLength;
    mCurrentLength = newCurrentLength;
    mSpringSettings = newSpringSettings;
    mConstraintPart = newConstraintPart;
}

// Add a component
void SpringJointComponents::addComponent(Entity jointEntity, bool isDisabled, const SpringJointComponent& component) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isDisabled);

    // Insert the new component data
    new (mJointEntities + index) Entity(jointEntity);
    mJoints[index] = nullptr;
    new (mLocalAnchorPointBody1 + index) Vector3(0, 0, 0);
    new (mLocalAnchorPointBody2 + index) Vector3(0, 0, 0);
    new (mR1World + index) Vector3(0, 0, 0);
    new (mR2World + index) Vector3(0, 0, 0);
    new (mAxisWorld + index) Vector3(0, 1, 0);
    new (mI1 + index) Matrix3x3();
    new (mI2 + index) Matrix3x3();
    mRestLength[index] = component.restLength;
    mCurrentLength[index] = component.restLength;
    new (mSpringSettings + index) SpringSettings(component.springSettings);
    new (mConstraintPart + index) AxisConstraintPart();

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(jointEntity, index));

    mNbComponents++;

    assert(mDisabledStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void SpringJointComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

    const Entity entity = mJointEntities[srcIndex];

    // Copy the data of the source component to the destination location
    new (mJointEntities + destIndex) Entity(mJointEntities[srcIndex]);
    mJoints[destIndex] = mJoints[srcIndex];
    new (mLocalAnchorPointBody1 + destIndex) Vector3(mLocalAnchorPointBody1[srcIndex]);
    new (mLocalAnchorPointBody2 + destIndex) Vector3(mLocalAnchorPointBody2[srcIndex]);
    new (mR1World + destIndex) Vector3(mR1World[srcIndex]);
    new (mR2World + destIndex) Vector3(mR2World[srcIndex]);
    new (mAxisWorld + destIndex) Vector3(mAxisWorld[srcIndex]);
    new (mI1 + destIndex) Matrix3x3(mI1[srcIndex]);
    new (mI2 + destIndex) Matrix3x3(mI2[srcIndex]);
    mRestLength[destIndex] = mRestLength[srcIndex];
    mCurrentLength[destIndex] = mCurrentLength[srcIndex];
    new (mSpringSettings + destIndex) SpringSettings(mSpringSettings[srcIndex]);
    new (mConstraintPart + destIndex) AxisConstraintPart(mConstraintPart[srcIndex]);

    // Destroy the source component
    destroyComponent(srcIndex);

    assert(!mMapEntityToComponentIndex.containsKey(entity));

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, destIndex));

    assert(mMapEntityToComponentIndex[mJointEntities[destIndex]] == destIndex);
}

// Swap two components in the array
void SpringJointComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity jointEntity1(mJointEntities[index1]);
    SpringJoint* joint1 = mJoints[index1];
    Vector3 localAnchorPointBody1(mLocalAnchorPointBody1[index1]);
    Vector3 localAnchorPointBody2(mLocalAnchorPointBody2[index1]);
    Vector3 r1World1(mR1World[index1]);
    Vector3 r2World1(mR2World[index1]);
    Vector3 axisWorld1(mAxisWorld[index1]);
    Matrix3x3 i11(mI1[index1]);
    Matrix3x3 i21(mI2[index1]);
    decimal restLength1 = mRestLength[index1];
    decimal currentLength1 = mCurrentLength[index1];
    SpringSettings springSettings1(mSpringSettings[index1]);
    AxisConstraintPart constraintPart1(mConstraintPart[index1]);

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mJointEntities + index2) Entity(jointEntity1);
    mJoints[index2] = joint1;
    new (mLocalAnchorPointBody1 + index2) Vector3(localAnchorPointBody1);
    new (mLocalAnchorPointBody2 + index2) Vector3(localAnchorPointBody2);
    new (mR1World + index2) Vector3(r1World1);
    new (mR2World + index2) Vector3(r2World1);
    new (mAxisWorld + index2) Vector3(axisWorld1);
    new (mI1 + index2) Matrix3x3(i11);
    new (mI2 + index2) Matrix3x3(i21);
    mRestLength[index2] = restLength1;
    mCurrentLength[index2] = currentLength1;
    new (mSpringSettings + index2) SpringSettings(springSettings1);
    new (mConstraintPart + index2) AxisConstraintPart(constraintPart1);

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(jointEntity1, index2));

    assert(mMapEntityToComponentIndex[mJointEntities[index1]] == index1);
    assert(mMapEntityToComponentIndex[mJointEntities[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void SpringJointComponents::destroyComponent(uint32 index) {

    Components::destroyComponent(index);

    assert(mMapEntityToComponentIndex[mJointEntities[index]] == index);

    mMapEntityToComponentIndex.remove(mJointEntities[index]);

    mJointEntities[index].~Entity();
    mJoints[index] = nullptr;
    mLocalAnchorPointBody1[index].~Vector3();
    mLocalAnchorPointBody2[index].~Vector3();
    mR1World[index].~Vector3();
    mR2World[index].~Vector3();
    mAxisWorld[index].~Vector3();
    mI1[index].~Matrix3x3();
    mI2[index].~Matrix3x3();
    mSpringSettings[index].~SpringSettings();
    mConstraintPart[index].~AxisConstraintPart();
}
