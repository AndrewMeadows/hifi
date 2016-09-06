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

#include <assert.h>

#include "CharacterRayResult.h"

const btScalar DEFAULT_STEP_UP_HEIGHT = 0.5f;


CharacterGhostObject::~CharacterGhostObject() {
    removeFromWorld();
    setCollisionShape(nullptr);
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


void CharacterGhostObject::setUpDirection(const btVector3& up) {
    btScalar length = up.length();
    if (length > FLT_EPSILON) {
        _upDirection /= length;
    } else {
        _upDirection = btVector3(0.0f, 1.0f, 0.0f);
    }
}

void CharacterGhostObject::setMotorVelocity(const btVector3& velocity) {
    _motorVelocity = velocity;
    _motorSpeed = _motorVelocity.length();
}

// override of btCollisionObject::setCollisionShape()
void CharacterGhostObject::setCollisionShape(btCollisionShape* shape) {
    assert(!shape || shape->isConvex()); // if shape is valid then please make it convex
    if (shape != getCollisionShape()) {
        bool wasInWorld = _inWorld;
        removeFromWorld();
        btCollisionObject::setCollisionShape(shape);
        if (wasInWorld) {
            assert(shape); // please remove from world before setting null shape
            addToWorld();
        }
    }
}

void CharacterGhostObject::setCollisionWorld(btCollisionWorld* world) {
    if (world != _world) {
        removeFromWorld();
        _world = world;
        addToWorld();
    }
}

void CharacterGhostObject::move(btScalar dt) {
    _onFloor = false;
    assert(_world && _inWorld);
    integrateKinematicMotion(dt);

    // resolve any penetrations before sweeping
    int32_t MAX_LOOPS = 4;
    int32_t numTries = 0;
    btVector3 totalPosition(0.0f, 0.0f, 0.0f);
    while (numTries < MAX_LOOPS) {
        if (resolvePenetration(numTries)) {
            numTries = 0;
            break;
        }
        totalPosition += getWorldTransform().getOrigin();
        ++numTries;
    }
    if (numTries > 1) {
        // penetration resolution was probably oscillating between opposing objects
        // so we use the average of the solutions
        totalPosition /= btScalar(numTries);
        btTransform transform = getWorldTransform();
        transform.setOrigin(totalPosition);
        setWorldTransform(transform);

        // TODO: figure out how to untrap character
    }
    if (_onFloor) {
        _hovering = false;
        updateTraction();
    }

    const btCollisionShape* shape = getCollisionShape();
    assert(shape->isConvex());
    const btConvexShape* convexShape= static_cast<const btConvexShape*>(shape);

    btVector3 sweepTranslation = dt * _linearVelocity;
    btScalar margin = shape->getMargin();
    if (numTries > 0 || _linearVelocity.length2() > margin * margin) {
        // update the pairCache for the sweepTests we intend to do
        btVector3 minAabb, maxAabb;
        getCollisionShape()->getAabb(getWorldTransform(), minAabb, maxAabb);
        minAabb.setMin(minAabb - btVector3(margin, margin, margin));
        maxAabb.setMax(maxAabb + btVector3(margin, margin, margin));
        minAabb.setMin(minAabb + sweepTranslation);
        maxAabb.setMax(maxAabb + sweepTranslation);
        minAabb.setMin(minAabb + _maxStepHeight * _upDirection);
        maxAabb.setMax(maxAabb + _maxStepHeight * _upDirection);

        // this updates both pairCaches: world broadphase and ghostobject
        _world->getBroadphase()->setAabb(getBroadphaseHandle(), minAabb, maxAabb, _world->getDispatcher());
    }

    // step forward
    CharacterSweepResult result(this);
    btTransform transform = getWorldTransform();
    btTransform nextTransform = transform;
    nextTransform.setOrigin(transform.getOrigin() + sweepTranslation);
    sweepTest(convexShape, transform, nextTransform, result); // forward

    if (!result.hasHit()) {
        transform = nextTransform;
        //setWorldTransform(nextTransform);
        updateHoverState(transform);
        updateTraction();
        return;
    }

    // update transform forward as much as possible
    transform.setOrigin(transform.getOrigin() + result.m_closestHitFraction * sweepTranslation);

    // truncate sweepTranslation to remainder
    sweepTranslation *= (1.0f - result.m_closestHitFraction);

    // early exit if this hit is obviously steppable
    btVector3 hitFromBase = result.m_hitPointWorld - (transform.getOrigin() - (_distanceToFeet * _upDirection));
    btScalar hitHeight = hitFromBase.dot(_upDirection);
    if (hitHeight > _maxStepHeight) {
        // capsule can't step over the obstacle so stop at hit
        //setWorldTransform(transform);
        return;
    }
    // if we get here then we hit something that might be steppable

    // sweep remainder after moving up availableStepHeight
    btScalar availableStepHeight = measureAvailableStepHeight();
    result.resetHitHistory();
    transform.setOrigin(transform.getOrigin() + availableStepHeight * _upDirection);
    nextTransform.setOrigin(transform.getOrigin() + sweepTranslation);
    sweepTest(convexShape, transform, nextTransform, result);
    if (result.hasHit()) {
        transform.setOrigin(transform.getOrigin() + result.m_closestHitFraction * sweepTranslation);
    } else {
        transform = nextTransform;
    }

    // sweep down in search of ground
    result.resetHitHistory();
    sweepTranslation = (dt * _linearVelocity.dot(_upDirection) - availableStepHeight) * _upDirection;
    nextTransform.setOrigin(transform.getOrigin() + sweepTranslation);
    sweepTest(convexShape, transform, nextTransform, result);
    if (result.hasHit()) {
        if (result.m_hitNormalWorld.dot(_upDirection) > _maxWallNormalUpComponent) {
            _floorNormal = result.m_hitNormalWorld;
            _onFloor = true;
            _hovering = false;
            transform.setOrigin(transform.getOrigin() + result.m_closestHitFraction * sweepTranslation);
        } else {
            // TODO? anything to do here?
        }
    } else {
        // sweep didn't hit anything
        transform = nextTransform;
        updateHoverState(transform);
        _onFloor = false;
    }
    updateTraction();
    //setWorldTransform(transform);
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
        //assert(getBroadphaseHandle());
        //int16_t group = getBroadphaseHandle()->m_collisionFilterGroup;
        //int16_t mask = getBroadphaseHandle()->m_collisionFilterMask;
        _world->addCollisionObject(this, _collisionFilterGroup, _collisionFilterMask);
        _inWorld = true;
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

        if (result.hasHit()) {
            return true;
        }
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

bool CharacterGhostObject::resolvePenetration(int numTries) {
    assert(_world);
    // We refresh the overlapping pairCache because any previous movement may have pushed us into an
    // overlap that was not in the cache.
    refreshOverlappingPairCache();

    // compute collision details
    btHashedOverlappingPairCache* pairCache = getOverlappingPairCache();
    _world->getDispatcher()->dispatchAllCollisionPairs(pairCache, _world->getDispatchInfo(), _world->getDispatcher());

    // loop over contact manifolds
    btTransform transform = getWorldTransform();
    btVector3 position = transform.getOrigin();
    btVector3 minBox =btVector3(0.0f, 0.0f, 0.0f);
    btVector3 maxBox = btVector3(0.0f, 0.0f, 0.0f);
    btManifoldArray manifoldArray;

    int numPairs = pairCache->getNumOverlappingPairs();
    for (int i = 0; i < numPairs; i++) {
        manifoldArray.resize(0);
        btBroadphasePair* collisionPair = &(pairCache->getOverlappingPairArray()[i]);

        btCollisionObject* obj0 = static_cast<btCollisionObject*>(collisionPair->m_pProxy0->m_clientObject);
        btCollisionObject* obj1 = static_cast<btCollisionObject*>(collisionPair->m_pProxy1->m_clientObject);

        if ((obj0 && !obj0->hasContactResponse()) && (obj1 && !obj1->hasContactResponse())) {
            // we know this probe has no contact response
            // but neither does the other object so skip this manifold
            continue;
        }

        if (!collisionPair->m_algorithm) {
            // null m_algorithm means the two shape types don't know how to collide!
            // shouldn't fall in here but just in case
            continue;
        }

        btScalar mostFloorPenetration = 0.0f;
        collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);
        for (int j = 0;j < manifoldArray.size(); j++) {
            btPersistentManifold* manifold = manifoldArray[j];
            btScalar directionSign = (manifold->getBody0() == this) ? btScalar(1.0) : btScalar(-1.0);
            for (int p = 0; p < manifold->getNumContacts(); p++) {
                const btManifoldPoint& pt = manifold->getContactPoint(p);
                if (pt.getDistance() > 0.0f) {
                    continue;
                }

                // normal always points from object to character
                btVector3 normal = directionSign * pt.m_normalWorldOnB;

                btScalar penetrationDepth = pt.getDistance();
                if (penetrationDepth < mostFloorPenetration) { // remember penetrationDepth is negative
                    btScalar normalDotUp = normal.dot(_upDirection);
                    if (normalDotUp > _maxWallNormalUpComponent) {
                        mostFloorPenetration = penetrationDepth;
                        _floorNormal = normal;
                        _onFloor = true;
                    }
                }

                btVector3 penetration = (-penetrationDepth + 0.0001f) * normal;
                minBox.setMin(penetration);
                maxBox.setMax(penetration);
            }
        }
    }

    btVector3 restore = maxBox + minBox;
    if (restore.length2() > 0.0f) {
        transform.setOrigin(position + restore);
        setWorldTransform(transform);
        return false;
    }
    return true;
}

void CharacterGhostObject::refreshOverlappingPairCache() {
    assert(_world && _inWorld);
    btVector3 minAabb, maxAabb;
    getCollisionShape()->getAabb(getWorldTransform(), minAabb, maxAabb);

    // this updates both pairCaches: world broadphase and ghostobject
    _world->getBroadphase()->setAabb(getBroadphaseHandle(), minAabb, maxAabb, _world->getDispatcher());
}

void CharacterGhostObject::integrateKinematicMotion(btScalar dt) {
    //btTransform transform = getWorldTransform();
    //transform.setOrigin(transform.getOrigin() + dt * _linearVelocity + (0.5f * dt * dt * _gravity) * _upDirection);
    //setWorldTransform(transform);
    _linearVelocity += (dt * _gravity) * _upDirection;
}

void CharacterGhostObject::updateTraction() {
    if (_hovering) {
        _linearVelocity = _motorVelocity;
    } else if (_onFloor) {
        btVector3 pathDirection = _floorNormal.cross(_motorVelocity).cross(_floorNormal);
        btScalar pathLength = pathDirection.length();
        if (pathLength > FLT_EPSILON) {
            _linearVelocity = (_motorSpeed / pathLength) * pathDirection;
        } else {
            _linearVelocity = btVector3(0.0f, 0.0f, 0.0f);
        }
    }
}

btScalar CharacterGhostObject::measureAvailableStepHeight() const {
    const btCollisionShape* shape = getCollisionShape();
    assert(shape->isConvex());
    const btConvexShape* convexShape= static_cast<const btConvexShape*>(shape);

    CharacterSweepResult result(this);
    btTransform transform = getWorldTransform();
    btTransform nextTransform = transform;
    nextTransform.setOrigin(transform.getOrigin() + _maxStepHeight * _upDirection);
    sweepTest(convexShape, transform, nextTransform, result);
    return result.m_closestHitFraction * _maxStepHeight;
}

void CharacterGhostObject::updateHoverState(const btTransform& transform) {
    // cast a ray down looking for floor support
    CharacterRayResult rayResult(this);
    btVector3 startPos = transform.getOrigin() - (_distanceToFeet * _upDirection);
    btVector3 endPos = startPos - (2.0f * _distanceToFeet) * _upDirection;
    rayTest(startPos, endPos, rayResult);
    // we're hovering if the ray didn't hit an object we can stand on
    _hovering = !(rayResult.hasHit() && rayResult.m_hitNormalWorld.dot(_upDirection) > _maxWallNormalUpComponent);
}

#if 0
void CharacterController::scanDown(btCollisionWorld* world) {
    BT_PROFILE("scanDown");
    // we test with downward raycast and if we don't find floor close enough then turn on "hover"
    btKinematicClosestNotMeRayResultCallback callback(_ghostObject);
    callback.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
    callback.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;

    btVector3 start = _currentPosition;
    const btScalar MAX_SCAN_HEIGHT = 20.0f + _halfHeight + _radius; // closest possible floor for disabling hover
    const btScalar MIN_HOVER_HEIGHT = 3.0f + _halfHeight + _radius; // distance to floor for enabling hover
    btVector3 end = start - MAX_SCAN_HEIGHT * _currentUp;

    world->rayTest(start, end, callback);
    if (!callback.hasHit()) {
        _isHovering = true;
    } else if (_isHovering && callback.m_closestHitFraction * MAX_SCAN_HEIGHT < MIN_HOVER_HEIGHT) {
        _isHovering = false;
    }
}

void CharacterController::stepUp(btCollisionWorld* world) {
    BT_PROFILE("stepUp");
    // phase 1: up

    // compute start and end
    btTransform start, end;
    start.setIdentity();
    start.setOrigin(_currentPosition + _currentUp * (_convexShape->getMargin() + _addedMargin));

    _targetPosition = _currentPosition + _currentUp * _stepUpHeight;
    end.setIdentity();
    end.setOrigin(_targetPosition);

    // sweep up
    btVector3 sweepDirNegative = - _currentUp;
    btKinematicClosestNotMeConvexResultCallback callback(_ghostObject, sweepDirNegative, btScalar(0.7071));
    callback.m_collisionFilterGroup = getGhostObject()->getBroadphaseHandle()->m_collisionFilterGroup;
    callback.m_collisionFilterMask = getGhostObject()->getBroadphaseHandle()->m_collisionFilterMask;
    _ghostObject->convexSweepTest(_convexShape, start, end, callback, world->getDispatchInfo().m_allowedCcdPenetration);

    if (callback.hasHit()) {
        // we hit something, so zero our vertical velocity
        _verticalVelocity = 0.0f;
        _verticalOffset = 0.0f;

        // Only modify the position if the hit was a slope and not a wall or ceiling.
        if (callback.m_hitNormalWorld.dot(_currentUp) > 0.0f) {
            _availableStepHeight = _stepUpHeight * callback.m_closestHitFraction;
            _currentPosition.setInterpolate3(_currentPosition, _targetPosition, callback.m_closestHitFraction);
        } else {
            _availableStepHeight = _stepUpHeight;
            _currentPosition = _targetPosition;
        }
    } else {
        _currentPosition = _targetPosition;
        _availableStepHeight = _stepUpHeight;
    }
}
#endif // FOO

