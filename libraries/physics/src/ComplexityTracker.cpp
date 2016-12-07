//
//  ComplexityTracker.cpp
//  libraries/physics/src
//
//  Created by Andrew Meadows 2016.11.10
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "ComplexityTracker.h"
#include <iostream> // adebug

#include <assert.h>

#include <PhysicsHelpers.h>

#include "ObjectMotionState.h"

ComplexityTracker::ComplexityTracker() : _state(ComplexityTracker::Inactive) {
}

ComplexityTracker::~ComplexityTracker() {
    clear();
}

void ComplexityTracker::clear() {
    _knownObjects.clear();
    clearQueue();
    _softMap.clear();
    _nextSoftExpiry = (uint64_t)(-1);
    _totalComplexity = 0;
    _quarantine.clear();
}

void ComplexityTracker::setSimulationStepRatio(float ratio) {
    _simulationStepRatio = ratio;
}

void ComplexityTracker::update(VectorOfMotionStates& changedObjects) {
    const uint64_t SETTLE_PERIOD = 2 * USECS_PER_SECOND;
    const float SLOW_RATIO = 0.25f;
    const float FAST_RATIO = 0.10f;
    const int32_t MAX_NUM_SLOW_STEPS = 8;

    // NOTE: a large value for _numSlowSteps is used to transition into Clamp state
    // while _lastEmergencyMeasureTimestep is used to transition into Release
    uint64_t now = usecTimestampNow();
    if (_simulationStepRatio > SLOW_RATIO) {
        if (_state == ComplexityTracker::Inactive) {
            ++_numSlowSteps;
            if (_numSlowSteps > MAX_NUM_SLOW_STEPS) {
                clear();
                _state = ComplexityTracker::EnabledButNotReady;
                _releaseExpiry = now + SETTLE_PERIOD;
            }
        } else if (_state == ComplexityTracker::Ready) {
            _state = ComplexityTracker::Clamp;
            std::cout << "adebug _state = Clamp 1" << std::endl;  // adebug
            _releaseExpiry = now + SETTLE_PERIOD;
        } else if (_state == ComplexityTracker::Release) {
            ++_numSlowSteps;
            if (_numSlowSteps > MAX_NUM_SLOW_STEPS) {
                _state = ComplexityTracker::Clamp;
            std::cout << "adebug _state = Clamp 3" << std::endl;  // adebug
                _releaseExpiry = now + SETTLE_PERIOD;
            }
        } else if (_state == ComplexityTracker::Clamp) {
            _releaseExpiry = now + SETTLE_PERIOD;
        }
    } else if (_simulationStepRatio < FAST_RATIO) {
        if (_state == ComplexityTracker::Inactive) {
            if (_numSlowSteps > 0) {
                --_numSlowSteps;
            }
        } else if (_state == ComplexityTracker::Release) {
            if (_quarantine.isEmpty() && _softMap.empty()) {
                _state = ComplexityTracker::Inactive;
                std::cout << "adebug _state = Inactive" << std::endl;  // adebug
                _numSlowSteps = 0;
            } else if (_numSlowSteps > 0) {
                --_numSlowSteps;
            }
        } else {
            if (_numSlowSteps > 0) {
                --_numSlowSteps;
            } else if (_releaseExpiry < now) {
                _state = ComplexityTracker::Release;
                std::cout << "adebug _state = Release 2" << std::endl;  // adebug
                _numSlowSteps = 0;
            }
        }
    }

    const int32_t STEPS_BETWEEN_CHANGES = 5;
    if ((_updateCounter % STEPS_BETWEEN_CHANGES) == 0) {
        // we only allow quarantine transitions every few steps
        // which gives the simulation some time to rebuild contact manifolds and resolve penetrations
        if (_state == ComplexityTracker::Clamp && _simulationStepRatio > FAST_RATIO) {
            // we are clamping down and the simulation is slow
            const float CLAMP_RATIO = 0.1f;
            int64_t target = (int64_t)(CLAMP_RATIO * (float)_totalQueueComplexity);
          	int64_t amount = 0L;
          	while(amount < target && _totalQueueComplexity > 0L) {
              	Complexity complexity = popTop();
                if (complexity.value > 0L) {
              	    _quarantine.insert(complexity);
              	    btRigidBody* body = complexity.key->getRigidBody();
                    int32_t bodyFlags = body->getCollisionFlags() | CF_QUARANTINE_SOFTEN_COLLISIONS;
                    uint32_t objectFlags = Simulation::DIRTY_MATERIAL;
                    QUuid ownerID = complexity.key->getSimulatorID();
                    if (Physics::getSessionUUID() != ownerID && !ownerID.isNull()) {
                        // the object is owned by someone else so we set it static
                        bodyFlags |= CF_QUARANTINE_SET_STATIC;
                        objectFlags |= Simulation::DIRTY_MOTION_TYPE;
                    }
              	    body->setCollisionFlags(bodyFlags);
              	    complexity.key->addDirtyFlags(objectFlags);
              	    changedObjects.push_back(complexity.key);
              	    amount += complexity.value;
                    std::cout << "adebug q+ " << (void*)(complexity.key) << "  " << complexity.value << std::endl;  // adebug
                }
          	}
            if (amount > 0L) {
                _releaseExpiry = now + SETTLE_PERIOD;
            }
        } else if (_state == ComplexityTracker::Release && _simulationStepRatio < FAST_RATIO) {
            int64_t target = 1L;
          	int64_t amount = 0L;
          	while(amount < target && !_quarantine.isEmpty()) {
                // we are releasing and the simulation is fast
              	Complexity complexity = _quarantine.bottom();
              	btRigidBody* body = complexity.key->getRigidBody();
                int32_t collisionFlags = body->getCollisionFlags();
                if (collisionFlags & CF_QUARANTINE_SET_STATIC) {
              	    body->setCollisionFlags(body->getCollisionFlags() & ~CF_QUARANTINE_SET_STATIC);
              	    complexity.key->addDirtyFlags(Simulation::DIRTY_MOTION_TYPE);
                }
                std::cout << "adebug q- " << (void*)(complexity.key) << "  " << complexity.value << std::endl;  // adebug
                _quarantine.pop();

                // add to the list of soft objects to help things settle
                uint64_t expiry = now + SETTLE_PERIOD;
                _softMap.insert({complexity.key, expiry});
                if (expiry < _nextSoftExpiry) {
                    _nextSoftExpiry = expiry;
                }

              	changedObjects.push_back(complexity.key);
              	amount += complexity.value;
          	}

            // release soft objects
            if (now > _nextSoftExpiry) {
                _nextSoftExpiry = (uint64_t)(-1);
                SoftMap::iterator softItr = _softMap.begin();
                while (softItr != _softMap.end()) {
                    uint64_t expiry = softItr->second;
                    if (expiry < now) {
              	        btRigidBody* body = softItr->first->getRigidBody();
                        int32_t collisionFlags = body->getCollisionFlags();
                        if (!body->isActive() && (collisionFlags & CF_QUARANTINE_SOFTEN_COLLISIONS) && !(collisionFlags & CF_QUARANTINE_SET_STATIC)) {
                            // flag object for material properties to be restored to normal
          	                body->setCollisionFlags(body->getCollisionFlags() & ~CF_QUARANTINE_SOFTEN_COLLISIONS);
          	                softItr->first->addDirtyFlags(Simulation::DIRTY_MATERIAL);
                            std::cout << "adebug s- " << (void*)(softItr->first) << std::endl;  // adebug
                            softItr = _softMap.erase(softItr);
                            // only restore one object at a time
                            _nextSoftExpiry = now + (uint64_t)((float)STEPS_BETWEEN_CHANGES * PHYSICS_ENGINE_FIXED_SUBSTEP * USECS_PER_SECOND);
                            break;
                        } else {
                            expiry = now + SETTLE_PERIOD;
                            softItr->second = expiry;
                        }
                    }
                    if (expiry < _nextSoftExpiry) {
                        _nextSoftExpiry = expiry;
                    }
                    ++softItr;
                }
            }
      	}
    }
    ++_updateCounter;
}

void ComplexityTracker::setInitialized() {
    assert(_state == ComplexityTracker::EnabledButNotReady);
    TrackerState oldState = _state;
    _state = ComplexityTracker::Ready;
    _updateCounter = 0;
}

void ComplexityTracker::remember(ObjectMotionState* key, int64_t value) {
    ComplexityMap::iterator itr = _knownObjects.find(key);
    _totalComplexity += value;
    if (itr == _knownObjects.end()) {
        _knownObjects.insert({key, value});
    } else {
        itr->second += value;
    }
    if (! (key->getRigidBody()->getCollisionFlags() & CF_QUARANTINE_FLAGS) ) {
        _queueIsDirty = true;
        _totalQueueComplexity += value;
    }
}

void ComplexityTracker::forget(ObjectMotionState* key, int64_t value) {
    ComplexityMap::iterator itr = _knownObjects.find(key);
    if (itr != _knownObjects.end()) {
        _totalComplexity -= value;
        if (itr->second > value) {
            itr->second -= value;
        } else {
            _knownObjects.erase(itr);
        }
        if (! (key->getRigidBody()->getCollisionFlags() & CF_QUARANTINE_FLAGS) ) {
            _queueIsDirty = true;
            _totalQueueComplexity -= value;
        }

        if (_knownObjects.empty()) {
            #ifdef DEBUG
            assert(_totalComplexity == 0);
            #else // DEBUG
            _totalComplexity = 0;
            #endif // DEBUG
        }
    }
}

void ComplexityTracker::remove(ObjectMotionState* key) {
    ComplexityMap::iterator itr = _knownObjects.find(key);
    if (itr != _knownObjects.end()) {
        _totalComplexity -= itr->second;
        if (! (itr->first->getRigidBody()->getCollisionFlags() & CF_QUARANTINE_FLAGS) ) {
            // this object was not yet taken for quarantine so _quarantineQueue will need to be rebuilt
            _totalQueueComplexity -= itr->second;
            _queueIsDirty = true;
        }
        _knownObjects.erase(itr);

        if (_knownObjects.empty()) {
            #ifdef DEBUG
            assert(_totalComplexity == 0);
            #else // DEBUG
            _totalComplexity = 0;
            #endif // DEBUG
        }
    }
    _quarantine.release(key);

    SoftMap::iterator softItr = _softMap.find(key);
    if (softItr != _softMap.end()) {
        _softMap.erase(softItr);
    }
}

Complexity ComplexityTracker::popTop() {
    Complexity complexity;
    if (_queueIsDirty) {
        rebuildQueue();
    }

    if (!_quarantineQueue.empty()) {
        complexity = _quarantineQueue.top();
        _totalQueueComplexity -= complexity.value;
        _quarantineQueue.pop();
    }
    return complexity;
}

void ComplexityTracker::clearQueue() {
    while (!_quarantineQueue.empty()) {
        _quarantineQueue.pop();
    }
    _queueIsDirty = true;
    _totalQueueComplexity = 0;
}

void ComplexityTracker::rebuildQueue() {
    // rebuild the queue of non-static, non-quarantined objects
    clearQueue();
    ComplexityMap::const_iterator itr = _knownObjects.begin();
    while (itr != _knownObjects.end()) {
        btRigidBody* body = itr->first->getRigidBody();
        if (! (body->getCollisionFlags() & CF_QUARANTINE_FLAGS) ) {
            _quarantineQueue.push(Complexity({itr->first, itr->second}));
            _totalQueueComplexity += itr->second;
        }
        ++itr;
    }
    _queueIsDirty = false;
}

