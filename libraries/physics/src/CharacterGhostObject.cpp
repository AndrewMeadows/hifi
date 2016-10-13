//
//  CharacterGhostObject.cpp
//  libraries/physcis/src
//
//  Created by Andrew Meadows 2016.08.26
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "CharacterGhostObject.h"

#include <stdint.h>
#include <assert.h>

#include <PhysicsHelpers.h>

#include "CharacterGhostShape.h"
#include "CharacterRayResult.h"


CharacterGhostObject::~CharacterGhostObject() {
    removeFromWorld();
    if (_ghostShape) {
        delete _ghostShape;
        _ghostShape = nullptr;
        setCollisionShape(nullptr);
    }
}

void CharacterGhostObject::setCollisionGroupAndMask(int16_t group, int16_t mask) {
    _collisionFilterGroup = group;
    _collisionFilterMask = mask;
    // TODO: if this probe is in the world reset ghostObject overlap cache
}

void CharacterGhostObject::getCollisionGroupAndMask(int16_t& group, int16_t& mask) const {
    group = _collisionFilterGroup;
    mask = _collisionFilterMask;
}

void CharacterGhostObject::setRadiusAndHalfHeight(btScalar radius, btScalar halfHeight) {
    _radius = radius;
    _halfHeight = halfHeight;
}

void CharacterGhostObject::setUpDirection(const btVector3& up) {
    btScalar length = up.length();
    if (length > FLT_EPSILON) {
        _upDirection /= length;
    } else {
        _upDirection = btVector3(0.0f, 1.0f, 0.0f);
    }
}

// override of btCollisionObject::setCollisionShape()
void CharacterGhostObject::setCharacterCapsule(btCapsuleShape* capsule) {
    assert(capsule);
    // we create our own CharacterGhostShape which has a larger Aabb for more reliable sweep tests
    if (_ghostShape) {
        delete _ghostShape;
    }
    _ghostShape = new CharacterGhostShape(capsule->getRadius(), 2.0f * capsule->getHalfHeight());
    setCollisionShape(_ghostShape);
}

void CharacterGhostObject::setCollisionWorld(btCollisionWorld* world) {
    if (world != _world) {
        removeFromWorld();
        _world = world;
        addToWorld();
    }
}

bool CharacterGhostObject::sweepTest(
        const btConvexShape* shape,
        const btTransform& start,
        const btTransform& end,
        CharacterSweepResult& result) const {
    if (_world && _inWorld) {
        assert(shape);
        btScalar allowedPenetration = _world->getDispatchInfo().m_allowedCcdPenetration;
        convexSweepTest(shape, start, end, result, allowedPenetration);
        return result.hasHit();
    }
    return false;
}

bool CharacterGhostObject::rayTest(const btVector3& start,
        const btVector3& end,
        CharacterRayResult& result) const {
    if (_world && _inWorld) {
        _world->rayTest(start, end, result);
    }
    return result.hasHit();
}

void CharacterGhostObject::refreshOverlappingPairCache() {
    assert(_world && _inWorld);
    btVector3 minAabb, maxAabb;
    getCollisionShape()->getAabb(getWorldTransform(), minAabb, maxAabb);
    // this updates both pairCaches: world broadphase and ghostobject
    _world->getBroadphase()->setAabb(getBroadphaseHandle(), minAabb, maxAabb, _world->getDispatcher());
}

btHashedOverlappingPairCache* CharacterGhostObject::computeOverlappingPairs() {
    refreshOverlappingPairCache();
    // compute collision details
    btHashedOverlappingPairCache* pairCache = getOverlappingPairCache();
    _world->getDispatcher()->dispatchAllCollisionPairs(pairCache, _world->getDispatchInfo(), _world->getDispatcher());

    return pairCache;
}

void CharacterGhostObject::removeFromWorld() {
    if (_world && _inWorld) {
        _world->removeCollisionObject(this);
        _inWorld = false;
    }
}

void CharacterGhostObject::addToWorld() {
    if (_world && !_inWorld) {
        assert(getCollisionShape());
        setCollisionFlags(getCollisionFlags() | btCollisionObject::CF_NO_CONTACT_RESPONSE);
        _world->addCollisionObject(this, _collisionFilterGroup, _collisionFilterMask);
        _inWorld = true;
    }
}

