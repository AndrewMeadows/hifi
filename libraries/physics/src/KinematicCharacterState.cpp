//
//  KinematicCharacterState.cpp
//  libraries/physcis/src
//
//  Created by Andrew Meadows 2016.08.26
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "KinematicCharacterState.h"

#include <stdint.h>
#include <assert.h>

#include <PhysicsHelpers.h>

#include "CharacterGhostObject.h"
#include "CharacterRayResult.h"


KinematicCharacterState::KinematicCharacterState(CharacterGhostObject* ghost) : _ghost(ghost) {
    assert(ghost);
}

KinematicCharacterState::~KinematicCharacterState() {
    _ghost = nullptr;
}

void KinematicCharacterState::setMotorVelocity(const btVector3& velocity) {
    _motorVelocity = velocity;
    if (_motorOnly) {
        _linearVelocity = _motorVelocity;
    }
}

void KinematicCharacterState::move(btScalar dt, btScalar overshoot, btScalar gravity) {
    bool oldOnFloor = _onFloor;
    _onFloor = false;
    _steppingUp = false;
    updateVelocity(dt, gravity);

    // resolve any penetrations before sweeping
    int32_t MAX_LOOPS = 4;
    int32_t numExtractions = 0;
    btVector3 totalPosition(0.0f, 0.0f, 0.0f);
    _ghost->setWorldTransform(_worldTransform);
    while (numExtractions < MAX_LOOPS) {
        if (resolvePenetration(numExtractions)) {
            numExtractions = 0;
            break;
        }
        totalPosition += _worldTransform.getOrigin();
        ++numExtractions;
    }
    if (numExtractions > 1) {
        // penetration resolution was probably oscillating between opposing objects
        // so we use the average of the solutions
        totalPosition /= btScalar(numExtractions);
        _worldTransform.setOrigin(totalPosition);
        _ghost->setWorldTransform(_worldTransform);

        // TODO: figure out how to untrap character
    }
    btVector3 startPosition = _worldTransform.getOrigin();
    if (_onFloor) {
        // resolvePenetration() pushed the avatar out of a floor so
        // we must updateTraction() before using _linearVelocity
        updateTraction(startPosition);
    }

    btScalar speed = _linearVelocity.length();
    btVector3 forwardSweep = dt * _linearVelocity;
    btScalar stepDistance = dt * speed;
    btScalar MIN_SWEEP_DISTANCE = 0.0001f;
    if (stepDistance < MIN_SWEEP_DISTANCE) {
        // not moving, no need to sweep
        updateTraction(startPosition);
        return;
    }

    const btCollisionShape* shape = _ghost->getCollisionShape();
    assert(shape->isConvex());
    const btConvexShape* convexShape= static_cast<const btConvexShape*>(shape);

    // augment forwardSweep to help slow moving sweeps get over steppable ledges
    btScalar margin = shape->getMargin();
    if (overshoot < margin) {
        overshoot = margin;
    }
    btScalar longSweepDistance = stepDistance + overshoot;
    forwardSweep *= longSweepDistance / stepDistance;

    // step forward
    CharacterSweepResult result(_ghost);
    btTransform nextTransform = _worldTransform;
    nextTransform.setOrigin(startPosition + forwardSweep);
    _ghost->sweepTest(convexShape, _worldTransform, nextTransform, result); // forward

    if (!result.hasHit()) {
        nextTransform.setOrigin(startPosition + (stepDistance / longSweepDistance) * forwardSweep);
        _worldTransform = nextTransform;
        updateTraction(nextTransform.getOrigin());
        return;
    }
    bool verticalOnly = btFabs(btFabs(_linearVelocity.dot(_ghost->getUpDirection())) - speed) < margin;
    if (verticalOnly) {
        // no need to step
        nextTransform.setOrigin(startPosition + (result.m_closestHitFraction * stepDistance / longSweepDistance) * forwardSweep);
        _worldTransform = nextTransform;

        if (result.m_hitNormalWorld.dot(_ghost->getUpDirection()) > _maxWallNormalUpComponent) {
            _floorNormal = result.m_hitNormalWorld;
            _floorContact = result.m_hitPointWorld;
            _steppingUp = false;
            _onFloor = true;
            _hovering = false;
        }
        updateTraction(nextTransform.getOrigin());
        return;
    }

    // check if this hit is obviously unsteppable
    btVector3 hitFromBase = result.m_hitPointWorld - (startPosition - ((0.5f * _ghost->getHeight()) * _ghost->getUpDirection()));
    btScalar hitHeight = hitFromBase.dot(_ghost->getUpDirection());
    if (hitHeight > _maxStepHeight) {
        // capsule can't step over the obstacle so move forward as much as possible before we bail
        btVector3 forwardTranslation = result.m_closestHitFraction * forwardSweep;
        btScalar forwardDistance = forwardTranslation.length();
        if (forwardDistance > stepDistance) {
            forwardTranslation *= stepDistance / forwardDistance;
        }
        _worldTransform.setOrigin(startPosition + forwardTranslation);
        _onFloor = _onFloor || oldOnFloor;
        return;
    }
    // if we get here then we hit something that might be steppable

    // remember the forward sweep hit fraction for later
    btScalar forwardSweepHitFraction = result.m_closestHitFraction;

    // figure out how high we can step up
    btScalar availableStepHeight = measureAvailableStepHeight();

    // raise by availableStepHeight before sweeping forward
    result.resetHitHistory();
    btTransform startTransform = _worldTransform;
    startTransform.setOrigin(startPosition + availableStepHeight * _ghost->getUpDirection());
    nextTransform.setOrigin(startTransform.getOrigin() + forwardSweep);
    _ghost->sweepTest(convexShape, startTransform, nextTransform, result);
    if (result.hasHit()) {
        startTransform.setOrigin(startTransform.getOrigin() + result.m_closestHitFraction * forwardSweep);
    } else {
        startTransform = nextTransform;
    }

    // sweep down in search of future landing spot
    result.resetHitHistory();
    btVector3 downSweep = (- availableStepHeight) * _ghost->getUpDirection();
    nextTransform.setOrigin(startTransform.getOrigin() + downSweep);
    _ghost->sweepTest(convexShape, startTransform, nextTransform, result);
    if (result.hasHit() && result.m_hitNormalWorld.dot(_ghost->getUpDirection()) > _maxWallNormalUpComponent) {
        // can stand on future landing spot, so we interpolate toward it
        _floorNormal = result.m_hitNormalWorld;
        _floorContact = result.m_hitPointWorld;
        _steppingUp = true;
        _onFloor = true;
        _hovering = false;
        nextTransform.setOrigin(startTransform.getOrigin() + result.m_closestHitFraction * downSweep);
        btVector3 totalStep = nextTransform.getOrigin() - startPosition;
        nextTransform.setOrigin(startPosition + (stepDistance / totalStep.length()) * totalStep);
        updateTraction(nextTransform.getOrigin());
    } else {
        // either there is no future landing spot, or there is but we can't stand on it
        // in any case: we go forward as much as possible
        nextTransform.setOrigin(startPosition + forwardSweepHitFraction * (stepDistance / longSweepDistance) * forwardSweep);
        _onFloor = _onFloor || oldOnFloor;
        updateTraction(nextTransform.getOrigin());
    }
    _worldTransform = nextTransform;
}

void KinematicCharacterState::updateVelocity(btScalar dt, btScalar gravity) {
    if (!_motorOnly) {
        if (_hovering) {
            _linearVelocity *= 0.999f; // HACK damping
        } else {
            _linearVelocity += (dt * gravity) * _ghost->getUpDirection();
        }
    }
}

void KinematicCharacterState::updateHoverState(const btVector3& position) {
    if (_onFloor) {
        _hovering = false;
    } else {
        // cast a ray down looking for floor support
        CharacterRayResult rayResult(_ghost);
        btScalar distanceToFeet = 0.5f * _ghost->getHeight();
        btScalar slop = 2.0f * _ghost->getCollisionShape()->getMargin(); // slop to help ray start OUTSIDE the floor object
        btVector3 startPos = position - ((distanceToFeet - slop) * _ghost->getUpDirection());
        btVector3 endPos = startPos - (2.0f * distanceToFeet) * _ghost->getUpDirection();
        _ghost->rayTest(startPos, endPos, rayResult);
        // we're hovering if the ray didn't hit anything or hit unstandable slope
        _hovering = !rayResult.hasHit() || rayResult.m_hitNormalWorld.dot(_ghost->getUpDirection()) < _maxWallNormalUpComponent;
    }
}

btScalar KinematicCharacterState::measureAvailableStepHeight() const {
    const btCollisionShape* shape = _ghost->getCollisionShape();
    assert(shape->isConvex());
    const btConvexShape* convexShape= static_cast<const btConvexShape*>(shape);

    CharacterSweepResult result(_ghost);
    btTransform nextTransform = _worldTransform;
    nextTransform.setOrigin(_worldTransform.getOrigin() + _maxStepHeight * _ghost->getUpDirection());
    _ghost->sweepTest(convexShape, _worldTransform, nextTransform, result);
    return result.m_closestHitFraction * _maxStepHeight;
}

void KinematicCharacterState::updateTraction(const btVector3& position) {
    updateHoverState(position);
    if (_hovering || _motorOnly) {
        _linearVelocity = _motorVelocity;
    } else if (_onFloor) {
        // compute a velocity that swings the capsule around the _floorContact
        btVector3 leverArm = _floorContact - position;
        btVector3 pathDirection = leverArm.cross(_motorVelocity.cross(leverArm));
        btScalar pathLength = pathDirection.length();
        if (pathLength > FLT_EPSILON) {
            _linearVelocity = (_motorVelocity.length() / pathLength) * pathDirection;
        } else {
            _linearVelocity = btVector3(0.0f, 0.0f, 0.0f);
        }
    }
}

void KinematicCharacterState::measurePenetration(btVector3& minBoxOut, btVector3& maxBoxOut) {
    // minBoxOut and maxBoxOut will be updated with penetration envelope.
    // If one of the corner points is <0,0,0> then the penetration is resolvable in a single step,
    // but if the space spanned by the two corners extends in both directions along at least one
    // component then we the object is sandwiched between two opposing objects.

    // We assume this object has just been moved to its current location, or else objects have been
    // moved around it since the last step so we must update the overlapping pairs.
    btHashedOverlappingPairCache* pairCache = _ghost->computeOverlappingPairs();

    // loop over contact manifolds to compute the penetration box
    minBoxOut = btVector3(0.0f, 0.0f, 0.0f);
    maxBoxOut = btVector3(0.0f, 0.0f, 0.0f);
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
            btScalar directionSign = (manifold->getBody0() == _ghost) ? btScalar(1.0) : btScalar(-1.0);
            for (int p = 0; p < manifold->getNumContacts(); p++) {
                const btManifoldPoint& pt = manifold->getContactPoint(p);
                if (pt.getDistance() > 0.0f) {
                    continue;
                }

                // normal always points from object to character
                btVector3 normal = directionSign * pt.m_normalWorldOnB;

                btScalar penetrationDepth = pt.getDistance();
                if (penetrationDepth < mostFloorPenetration) { // remember penetrationDepth is negative
                    btScalar normalDotUp = normal.dot(_ghost->getUpDirection());
                    if (normalDotUp > _maxWallNormalUpComponent) {
                        mostFloorPenetration = penetrationDepth;
                        _floorNormal = normal;
                        if (directionSign > 0.0f) {
                            _floorContact = pt.m_positionWorldOnA;
                        } else {
                            _floorContact = pt.m_positionWorldOnB;
                        }
                        _onFloor = true;
                    }
                }

                btVector3 penetration = (-penetrationDepth) * normal;
                minBoxOut.setMin(penetration);
                maxBoxOut.setMax(penetration);
            }
        }
    }
}

bool KinematicCharacterState::resolvePenetration(int numTries) {
    btVector3 minBox, maxBox;
    measurePenetration(minBox, maxBox);
    btVector3 restore = maxBox + minBox;
    if (restore.length2() > 0.0f) {
        _worldTransform.setOrigin(_worldTransform.getOrigin() + restore);
        _ghost->setWorldTransform(_worldTransform);
        return false;
    }
    return true;
}
