//
//  SweepProbe.cpp
//  libraries/physcis/src
//
//  Created by Andrew Meadows 2016.08.26
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "SweepProbe.h"

#include <iostream> // adebug
#include <assert.h>


const btVector3 ZERO_VEC(0.0f, 0.0f, 0.0f);
const btScalar MIN_PROBE_RADIUS = 0.001f;
const btScalar MAX_PROBE_RADIUS = 100.0f;

class SweepResult : public btCollisionWorld::ClosestConvexResultCallback {
public:
    SweepResult(SweepProbe* probe)
    :   btCollisionWorld::ClosestConvexResultCallback(ZERO_VEC, ZERO_VEC),
        _probe(probe)
    {
        // set collision group and mask to match _probe
        assert(_probe);
        _probe->getCollisionGroupAndMask(m_collisionFilterGroup, m_collisionFilterMask);
        //m_collisionFilterGroup = _probe->getBroadphaseHandle()->m_collisionFilterGroup;
        //m_collisionFilterMask = _probe->getBroadphaseHandle()->m_collisionFilterMask;
    }

    virtual btScalar addSingleResult(btCollisionWorld::LocalConvexResult& convexResult, bool useWorldFrame) override {
        if (convexResult.m_hitCollisionObject == _probe) {
            return btScalar(1.0);
        }

        if (!convexResult.m_hitCollisionObject->hasContactResponse()) {
            return btScalar(1.0);
        }

        btVector3 hitNormalWorld;
        if (useWorldFrame) {
            hitNormalWorld = convexResult.m_hitNormalLocal;
        } else {
            ///need to transform normal into worldspace
            hitNormalWorld = convexResult.m_hitCollisionObject->getWorldTransform().getBasis() * convexResult.m_hitNormalLocal;
        }

        return ClosestConvexResultCallback::addSingleResult(convexResult, useWorldFrame);
    }

protected:
    SweepProbe* _probe;
};


SweepProbe::~SweepProbe() {
    removeFromWorld();
    setCollisionShape(nullptr);
}

void SweepProbe::getCollisionGroupAndMask(int16_t& group, int16_t& mask) {
    group = _collisionFilterGroup;
    mask = _collisionFilterMask;
}


void SweepProbe::setOverlapMargin(float margin) {
    _overlapMargin = margin;
}

/*
void SweepProbe::setRadius(btScalar radius) {
    if (radius < MIN_PROBE_RADIUS) {
        radius = MIN_PROBE_RADIUS;
    } else if (radius > MAX_PROBE_RADIUS) {
        radius = MAX_PROBE_RADIUS;
    }
    if (_radius != radius) {
        _radius = radius;
        bool wasInWorld = _inWorld;
        removeFromWorld();
        updateShape();
        if (wasInWorld) {
            addToWorld();
        }
    }
}

void SweepProbe::setLocalAabb(const btVector3& aabbMin, const btVector3& aabbMax) {
    _localAabbMin = aabbMin;
    _localAabbMax = aabbMax;
}
*/

// override of btCollisionObject::setCollisionShape()
void SweepProbe::setCollisionShape(btCollisionShape* shape) {
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

void SweepProbe::setCollisionFilterGroupAndMask(int16_t group, int16_t mask) {
    _collisionFilterGroup = group;
    _collisionFilterMask = mask;
    // TODO: if this probe is in the world reset ghostObject overlap cache
}

void SweepProbe::setCollisionWorld(btCollisionWorld* world) {
    if (world != _world) {
        bool wasInWorld = _inWorld;
        removeFromWorld();
        _world = world;
        addToWorld();
    }
}

void SweepProbe::refreshOverlappingPairCache() {
    assert(_world && _inWorld);
    // TODO? use custom expanded Aabb?
    btVector3 minAabb, maxAabb;
    getCollisionShape()->getAabb(getWorldTransform(), minAabb, maxAabb);
    btVector3 margin = btVector3(_overlapMargin, _overlapMargin, _overlapMargin);
    minAabb -= margin;
    maxAabb += margin;

    // this updates both paircaches: world broadphase and ghostobject
    _world->getBroadphase()->setAabb(getBroadphaseHandle(), minAabb, maxAabb, _world->getDispatcher());
}

void SweepProbe::measurePenetration() {
    /*
    assert(_world);
    // We refresh the overlapping paircache because any previous movement may have pushed us into an
    // overlap that was not in the cache.
    refreshOverlappingPairCache();

    // compute collision details
    btHashedOverlappingPairCache* pairCache = getOverlappingPairCache();
    _world->getDispatcher()->dispatchAllCollisionPairs(pairCache, _world->getDispatchInfo(), _world->getDispatcher());

    // loop over contact manifolds
    btVector3 oldPosition = getWorldTransform().getOrigin();
    btVector3 newPosition = oldPosition;
    btVector3 minPenetration(0.0f, 0.0f, 0.0f);
    btVector3 maxPenetration(0.0f, 0.0f, 0.0f);
    btManifoldArray manifoldArray;

    int numPairs = pairCache->getNumOverlappingPairs();
    for (int i = 0; i < numPairs; i++) {
        manifoldArray.resize(0);
        btBroadphasePair* collisionPair = &(pairCache->getOverlappingPairArray()[i]);

        btCollisionObject* obj0 = static_cast<btCollisionObject*>(collisionPair->m_pProxy0->m_clientObject);
        btCollisionObject* obj1 = static_cast<btCollisionObject*>(collisionPair->m_pProxy1->m_clientObject);

        if ((obj0 && !obj0->hasContactResponse()) || (obj1 && !obj1->hasContactResponse())) {
            continue;
        }

        if (collisionPair->m_algorithm) {
            collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);
        }

        for (int j = 0;j < manifoldArray.size(); j++) {
            btPersistentManifold* manifold = manifoldArray[j];
            btScalar directionSign = (manifold->getBody0() == this) ? btScalar(1.0) : btScalar(-1.0);
            for (int p = 0; p < manifold->getNumContacts(); p++) {
                const btManifoldPoint&pt = manifold->getContactPoint(p);
                // BOOKMARK -- measure penetration envelope as an AABB box
                // might actually need to salvage the maxSlopeCosine stuff for walking up slopes

                btScalar dist = pt.getDistance();
                if (dist < 0.0f) {
                    btVector3 normal = pt.m_normalWorldOnB;
                    normal *= directionSign; // always points from object to character

                    btScalar normalDotUp = normal.dot(_currentUp);
                    if (normalDotUp < _maxSlopeCosine) {
                        // this contact has a non-vertical normal... might need to ignored
                        btVector3 collisionPoint;
                        if (directionSign > 0.0) {
                            collisionPoint = pt.getPositionWorldOnB();
                        } else {
                            collisionPoint = pt.getPositionWorldOnA();
                        }

                        // we do math in frame where character base is origin
                        btVector3 characterBase = oldPosition - (_radius + _halfHeight) * _currentUp;
                        collisionPoint -= characterBase;
                        btScalar collisionHeight = collisionPoint.dot(_currentUp);

                        if (collisionHeight < _lastStepUp) {
                            // This contact is below the lastStepUp, so we ignore it for penetration resolution,
                            // otherwise it may prevent the character from getting close enough to find any available
                            // horizontal foothold that would allow it to climbe the ledge.  In other words, we're
                            // making the character's "feet" soft for collisions against steps, but not floors.
                            useContact = false;
                        }
                    }
                    if (useContact) {

                        if (dist < maxPen) {
                            maxPen = dist;
                            _floorNormal = normal;
                        }
                        const btScalar INCREMENTAL_RESOLUTION_FACTOR = 0.2f;
                        newPosition += normal * (fabsf(dist) * INCREMENTAL_RESOLUTION_FACTOR);
                    }
                }
            }
        }
    }
    btTransform newTrans = getWorldTransform();
    newTrans.setOrigin(newPosition);
    setWorldTransform(newTrans);
    */
}

bool SweepProbe::sweep(const btConvexShape* shape, const btTransform& start, const btTransform& end) {
    if (_world && _inWorld) {
        assert(shape);
        SweepResult result(this);

        btScalar allowedPenetration = _world->getDispatchInfo().m_allowedCcdPenetration;
        convexSweepTest(shape, start, end, result, allowedPenetration);

        if (result.hasHit()) {
            btVector3 n = result.m_hitNormalWorld;
            std::cout << "adebug sweep fraction = " << result.m_closestHitFraction
                << "  normal = <" << n.getX() << ", " << n.getY() << ", " << n.getZ() << ">"
                << std::endl;  // adebug
            return true;
        }
    }
    return false;
}

void SweepProbe::removeFromWorld() {
    if (_world && _inWorld) {
        _world->removeCollisionObject(this);
        _inWorld = false;
    }
}

void SweepProbe::addToWorld() {
    if (_world && !_inWorld) {
        assert(getCollisionShape());
        //assert(getBroadphaseHandle());
        //int16_t group = getBroadphaseHandle()->m_collisionFilterGroup;
        //int16_t mask = getBroadphaseHandle()->m_collisionFilterMask;
        _world->addCollisionObject(this, _collisionFilterGroup, _collisionFilterMask);
        _inWorld = true;
    }
}

/*
void SweepProbe::updateShape() {
    assert(!_inWorld);
    assert(_radius >= MIN_PROBE_RADIUS);
    assert(_radius <= MAX_PROBE_RADIUS);
    if (_shape) {
        delete _shape;
        _shape = nullptr;
    }
    _shape = new btSphereShape(_radius);
    setCollisionShape(_shape);
}
*/


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
            _lastStepUp = _stepUpHeight * callback.m_closestHitFraction;
            _currentPosition.setInterpolate3(_currentPosition, _targetPosition, callback.m_closestHitFraction);
        } else {
            _lastStepUp = _stepUpHeight;
            _currentPosition = _targetPosition;
        }
    } else {
        _currentPosition = _targetPosition;
        _lastStepUp = _stepUpHeight;
    }
}
#endif // FOO

