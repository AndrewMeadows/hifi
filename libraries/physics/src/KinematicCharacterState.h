//
//  KinematicCharacterState.h
//  libraries/physcis/src
//
//  Created by Andrew Meadows 2016.08.26
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_KinematicCharacterState_h
#define hifi_KinematicCharacterState_h

#include <assert.h>
#include <stdint.h>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <stdint.h>

class CharacterGhostObject;

class KinematicCharacterState {
public:
    KinematicCharacterState(CharacterGhostObject* ghost);
    ~KinematicCharacterState();

    void setWorldTransform(const btTransform& transform) { _worldTransform = transform; }
    const btTransform& getWorldTransform() const { return _worldTransform; }

    void setMotorVelocity(const btVector3& velocity);

    void setLinearVelocity(const btVector3& velocity) { _linearVelocity = velocity; }
    const btVector3& getLinearVelocity() const { return _linearVelocity; }

    void setMaxStepHeight(btScalar height) { _maxStepHeight = height; }
    //btScalar getMaxStepHeight() const { return _maxStepHeight; }
    void setMinWallAngle(btScalar angle) { _maxWallNormalUpComponent = cosf(angle); }

    void move(btScalar dt, btScalar overshoot, btScalar gravity);

    bool isHovering() const { return _hovering; }
    void setHovering(bool hovering) { _hovering = hovering; }
    void setMotorOnly(bool motorOnly) { _motorOnly = motorOnly; }

    bool hasSupport() const { return _onFloor; }
    bool isSteppingUp() const { return _steppingUp; }
    const btVector3& getFloorNormal() const { return _floorNormal; }

    void measurePenetration(btVector3& minBoxOut, btVector3& maxBoxOut);

protected:
    bool resolvePenetration(int numTries);
    void updateVelocity(btScalar dt, btScalar gravity);
    void updateTraction(const btVector3& position);
    void updateHoverState(const btVector3& position);
    btScalar measureAvailableStepHeight() const;

protected:
    btTransform _worldTransform;
    btVector3 _motorVelocity { 0.0f, 0.0f, 0.0f }; // input, velocity character is trying to achieve
    btVector3 _linearVelocity { 0.0f, 0.0f, 0.0f }; // internal, actual character velocity
    btVector3 _floorNormal { 0.0f, 0.0f, 0.0f }; // internal, probable floor normal
    btVector3 _floorContact { 0.0f, 0.0f, 0.0f }; // internal, last floor contact point
    btScalar _maxWallNormalUpComponent { 0.0f }; // input: max vertical component of wall normal
    btScalar _maxStepHeight { 0.0f }; // input, max step height the character can climb
    CharacterGhostObject* _ghost { nullptr }; // input, ghost for performing sweep/ray tests
    bool _hovering { false }; // internal,
    bool _onFloor { false }; // output, is actually standing on floor
    bool _steppingUp { false }; // output, future sweep hit a steppable ledge
    bool _hasFloor { false }; // output, has floor underneath to fall on
    bool _motorOnly { false }; // input, _linearVelocity slaves to _motorVelocity
};

#endif // hifi_KinematicCharacterState_h
