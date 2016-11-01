//
//  SettleAction.cpp
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
#include <iostream> // adebug

#include <PhysicsHelpers.h>
#include <SharedUtil.h>

// One problem with a remote-authoritative distributed physics simulation is that local objects in an dynamic
// pile can sometimes be teleported into penetration with their neighbors, e.g. when their collision neighbors
// have not been updated yet.  The solution is to ease objects into their new transform over finite time rather
// than teleporting them directly.  This is especially problematic for Hifi when objects are settling down
// because fewer updates are sent for objects that are not moving fast and whose positions don't differ much
// from the last update sent to the entity-server.  To solve this problem we introduce this SettleAction
// which will try to ease objects into their final resting place without letting them exceed their sleeping
// threshold speeds:
//
// Bullet will deactivate objects that have speeds below sleeping thresholds for some "deactivation time".
// We set these thresholds to custom values, but the deactivation time is hard coded by Bullet:
//
// DYNAMIC_LINEAR_SPEED_THRESHOLD = 0.05 m/sec
// DYNAMIC_ANGULAR_SPEED_THRESHOLD = 0.087266f deg/sec (~5 deg/sec)
// btRigidBody::gDeactivationTime = 2.0 sec
//
// This means there is a maximum distance over which the object can be eased.  For the linear case:
//
// MAX_EASE_DISTANCE = DYNAMIC_LINEAR_SPEED_THRESHOLD * btRigidBody::gDeactivationTime = 0.20 m

// local helper function
void slamBodyToTarget(btRigidBody* body, const btTransform& targetTransform) {
    const btVector3 zero(0.0f, 0.0f, 0.0f);
    body->setLinearVelocity(zero);
    body->setAngularVelocity(zero);
    body->setWorldTransform(targetTransform);
}

// local helper function
void slaveBodyToTarget(btRigidBody* body, const btTransform& targetTransform) {
    return;
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
        // The object won't be able to reach its target within timeRemaining while keeping below
        // the deactivation thresholds, so we micro-teleport close enought to the target to make
        // it possible for next frame.
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
        // micro-teleport for angular displacement
        angle = maxEaseAngle;
        dQ.setRotation(dQ.getAxis(), angle);
        // Q2 = dQ * Q1  -->  Q1 = dQ^ * Q2
        currentTransform.setRotation(dQ.inverse() * targetTransform.getRotation());
    }
    body->setAngularVelocity((-angle / timeRemaining) * dQ.getAxis());

    body->setWorldTransform(currentTransform);
}

uint64_t appStart; // adebug

// helper function
float secondsSinceStart() {
    uint32_t foo = uint32_t((usecTimestampNow() - appStart) / 1000);
    return (float)(foo) / 1000.0f;
}

SettleAction::SettleAction(uint8_t maxNumBodies) {
    const uint8_t MAX_MAX_NUM_BODIES = 254;
    assert(maxNumBodies < MAX_MAX_NUM_BODIES);
    assert(maxNumBodies > 0);
    _maxNumBodies = maxNumBodies;
    _body = new btRigidBody* [_maxNumBodies];
    _transform = new btTransform [_maxNumBodies];
    _expiry = new uint64_t [_maxNumBodies];

    appStart = usecTimestampNow(); // adebug
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
        std::cout << "adebug " << secondsSinceStart() << "  SettleAction::addBody(" << (void*)body << ")  N = " << (int)_numBodies << std::endl;  // adebug
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
    for (uint8_t i = 0; i < _numBodies; ++i) {
        if (_body[i] == body) {
            uint8_t lastIndex = _numBodies - 1;
            if (i < lastIndex) {
                _body[i] = _body[lastIndex];
                _transform[i] = _transform[lastIndex];
                _expiry[i] = _expiry[lastIndex];
            }
            --_numBodies;
            std::cout << "adebug " << secondsSinceStart() << "  SettleAction::removeBody(" << (void*)body << ")  N = " << (int)_numBodies << std::endl;  // adebug
            break;
        }
    }
}

void SettleAction::updateAction(btCollisionWorld* collisionWorld, btScalar deltaTimeStep) {
    uint64_t now = usecTimestampNow();
    uint8_t i = _numBodies;
    while (i > 0) {
        --i;
        if (now > _expiry[i]) {
            if (_body[i]->isActive()) {
                // the body is still active, but we're out of time
                slamBodyToTarget(_body[i], _transform[i]);
            }

            // remove this one
            uint8_t lastIndex = _numBodies - 1;
            --_numBodies;
            std::cout << "adebug " << secondsSinceStart() << "  SettleAction::expireBody(" << (void*)_body[i] << ")  N = " << (int)_numBodies << std::endl;  // adebug
            if (i < lastIndex) {
                _body[i] = _body[lastIndex];
                _transform[i] = _transform[lastIndex];
                _expiry[i] = _expiry[lastIndex];
            }
        } else if (_body[i]->isActive()) {
            slaveBodyToTarget(_body[i], _transform[i]);
        }
    }
}

