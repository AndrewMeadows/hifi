//
//  CharacterGhostObject.h
//  libraries/physcis/src
//
//  Created by Andrew Meadows 2016.08.26
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_CharacterGhostObject_h
#define hifi_CharacterGhostObject_h

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

#include "CharacterSweepResult.h"
#include "CharacterRayResult.h"


class CharacterGhostObject : public btPairCachingGhostObject {
public:
    CharacterGhostObject() { }
    ~CharacterGhostObject();

    void setCollisionGroupAndMask(int16_t group, int16_t mask);
    void getCollisionGroupAndMask(int16_t& group, int16_t& mask) const;

    void setDistanceToFeet(btScalar distance) { _distanceToFeet = distance; }
    void setUpDirection(const btVector3& up);
    void setMotorVelocity(const btVector3& velocity);
    void setGravity(btScalar gravity) { _gravity = gravity; } // NOTE: we expect _gravity to be negative (in _upDirection)
    void setMinWallAngle(btScalar angle) { _maxWallNormalUpComponent = cosf(angle); }
    void setMaxStepHeight(btScalar height) { _maxStepHeight = height; }

    void setCollisionShape(btCollisionShape* shape) override;

    void setCollisionWorld(btCollisionWorld* world);

    void move(btScalar dt);

protected:
    void removeFromWorld();
    void addToWorld();

    bool sweepTest(const btConvexShape* shape,
            const btTransform& start,
            const btTransform& end,
            CharacterSweepResult& result) const;
    bool rayTest(const btVector3& start,
            const btVector3& end,
            CharacterRayResult& result) const;

    bool resolvePenetration(int numTries);
    void refreshOverlappingPairCache();
    void integrateKinematicMotion(btScalar dt);
    void updateTraction();
    btScalar measureAvailableStepHeight() const;
    void updateHoverState(const btTransform& transform);

protected:
    btVector3 _upDirection { 0.0f, 1.0f, 0.0f }; // input, up in world-frame
    btVector3 _motorVelocity { 0.0f, 0.0f, 0.0f }; // input, velocity character is trying to achieve
    btVector3 _linearVelocity { 0.0f, 0.0f, 0.0f }; // internal, actual character velocity
    btVector3 _floorNormal { 0.0f, 0.0f, 0.0f }; // internal, probable floor normal
    btCollisionWorld* _world { nullptr }; // input, pointer to world
    btScalar _distanceToFeet { 0.0f }; // input, distance from object center to lowest point on shape
    btScalar _motorSpeed { 0.0f }; // internal, cached for speed
    btScalar _gravity { 0.0f }; // input, amplitude of gravity along _upDirection (should be negative)
    btScalar _maxWallNormalUpComponent { 0.0f }; // input: max vertical component of wall normal
    btScalar _maxStepHeight { 0.0f }; // input, max step height the character can climb
    int16_t _collisionFilterGroup { 0 };
    int16_t _collisionFilterMask { 0 };
    bool _inWorld { false }; // internal, was added to world
    bool _hovering { false }; // internal, 
    bool _onFloor { false }; // output, is actually standing on floor
    bool _hasFloor { false }; // output, has floor underneath to fall on
};

#endif // hifi_CharacterGhostObject_h
