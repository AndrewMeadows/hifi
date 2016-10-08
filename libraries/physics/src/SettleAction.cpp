//
//  SettleAction.h
//  libraries/physcis/src
//
//  Created by Andrew Meadows 2016-10-06
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//
//  http://bulletphysics.org/Bullet/BulletFull/classbtActionInterface.html

#include "SettleAction.h"

#include <PhysicsHelpers.h>
#include <SharedUtil.h>

// The problem is that objects are sometimes teleported into penetration while trying to settle, e.g. when
// their collision neighbors not have been updated yet.  The solution is to ease objects into their settling
// transform over time rather than teleporting them directly.  Since the object is settling we don't want
// its velocities to exceed the sleeping threshold speeds during the deactivation time:
//
// DYNAMIC_LINEAR_SPEED_THRESHOLD = 0.05 m/sec
// DYNAMIC_ANGULAR_SPEED_THRESHOLD = 0.087266f deg/sec (~5 deg/sec)
// btRigidBody::gDeactivationTime = 2.0 sec  <-- hard coded in btRigidBody.cpp
//
// This means there is a maximum distance over which the object can be eased.  For the linear case:
//
// MAX_EASE_DISTANCE = DYNAMIC_LINEAR_SPEED_THRESHOLD * btRigidBody::gDeactivationTime = 0.20 m

const btScalar MAX_DEACTIVATION_TIME = 2.0f;

void slamBodyToTarget(btRigidBody* body, const btTransform& targetTransform) {
    // yes we can teleport directly
    const btVector3 zero(0.0f, 0.0f, 0.0f);
    body->setLinearVelocity(zero);
    body->setAngularVelocity(zero);
    body->setWorldTransform(targetTransform);
}

void slaveBodyToTarget(btRigidBody* body, const btTransform& targetTransform) {
    btScalar timeRemaining = MAX_DEACTIVATION_TIME - body->getDeactivationTime();
    if (timeRemaining < 0.0f) {
        slamBodyToTarget(body, targetTransform);
        return;
    }

    btTransform currentTransform = body->getWorldTransform();

    // linear
    btVector3 offset = targetTransform.getOrigin() - currentTransform.getOrigin();
    btScalar distance = offset.length();
    const btScalar maxEaseDistance = DYNAMIC_LINEAR_SPEED_THRESHOLD * timeRemaining;
    if (distance > maxEaseDistance) {
        timeRemaining = distance / DYNAMIC_LINEAR_SPEED_THRESHOLD;
        if (timeRemaining > MAX_DEACTIVATION_TIME) {
            timeRemaining = MAX_DEACTIVATION_TIME;
        }
        offset *= (DYNAMIC_LINEAR_SPEED_THRESHOLD * timeRemaining) / distance;
        currentTransform.setOrigin(targetTransform.getOrigin() - offset);
    }
    body->setLinearVelocity(offset / MAX_DEACTIVATION_TIME);

    // angular
    // Q2 = dQ * Q1  -->  dQ = Q2 * Q1^
    btQuaternion dQ = targetTransform.getRotation() * currentTransform.getRotation().inverse();
    btScalar angle = dQ.getAngle();
    const btScalar maxEaseAngle = DYNAMIC_ANGULAR_SPEED_THRESHOLD * timeRemaining;
    if (angle > maxEaseAngle) {
        angle = maxEaseAngle;
        dQ.setRotation(dQ.getAxis(), angle);
        // Q2 = dQ * Q1  -->  Q1 = dQ^ * Q2
        currentTransform.setRotation(dQ.inverse() * targetTransform.getRotation());
    }
    body->setAngularVelocity((-angle / timeRemaining) * dQ.getAxis());

    body->setWorldTransform(currentTransform);
}

SettleAction::SettleAction(uint8_t maxNumBodies) {
    const uint8_t MAX_MAX_NUM_BODIES = 254;
    assert(maxNumBodies < MAX_MAX_NUM_BODIES);
    assert(maxNumBodies > 0);
    _body = new btRigidBody* [_maxNumBodies];
    _transform = new btTransform [_maxNumBodies];
    _expiry = new uint64_t [_maxNumBodies];
    _maxNumBodies = maxNumBodies;
}

SettleAction::~SettleAction() {
    delete [] _body;
    delete [] _transform;
    delete [] _expiry;
    _body = nullptr;
    _transform = nullptr;
    _expiry = nullptr;
    _numBodies = 0;
}

void SettleAction::addBody(btRigidBody* body, const btTransform& targetTransform, uint64_t expiry) {
    assert(body);

    // look for the body...
    uint8_t index = 0;
    for (index = 0; index < _numBodies; ++index) {
        if (body == _body[index]) {
            break;
        }
    }

    if (index == _maxNumBodies) {
        // the body is not in the list and the list is full --> we can't help
        return;
    }

    if (index == _numBodies) {
        // the body is not in the list but there is room --> add it
        ++_numBodies;
        _body[index] = body;
    }
    _transform[index] = targetTransform;
    _expiry[index] = expiry;

    if (body->isActive()) {
        slaveBodyToTarget(body, targetTransform);
    } else {
        body->setWorldTransform(targetTransform);
    }
}

void SettleAction::removeBody(btRigidBody* body) {
    assert(body);
    uint8_t lastIndex = _numBodies - 1;
    for (uint8_t i = 0; i < _numBodies; ++i) {
        if (_body[i] == body) {
            if (i < lastIndex) {
                _body[i] = _body[lastIndex];
                _transform[i] = _transform[lastIndex];
                _expiry[i] = _expiry[lastIndex];
            }
            --_numBodies;
            break;
        }
    }
}

void SettleAction::updateAction(btCollisionWorld* collisionWorld, btScalar deltaTimeStep) {
    uint64_t now = usecTimestampNow();
    uint8_t i = _numBodies;
    while (i > 0) {
        --i;
        btRigidBody* body = _body[i];
        if (!body->isActive()) {
            continue;
        }

        if (now > _expiry[i]) {
            // the body isn't inactive yet, but we're out of time
            slamBodyToTarget(_body[i], _transform[i]);

            uint8_t lastIndex = _numBodies - 1;
            _body[i] = _body[lastIndex];
            _transform[i] = _transform[lastIndex];
            _expiry[i] = _expiry[lastIndex];
            --_numBodies;
            continue;
        }

        slaveBodyToTarget(_body[i], _transform[i]);
    }
}

