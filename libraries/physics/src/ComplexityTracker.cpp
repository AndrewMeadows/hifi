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

#include <assert.h>

#include "ObjectMotionState.h"

ComplexityTracker::ComplexityTracker() : _state(ComplexityTracker::Inactive) {
}

ComplexityTracker::~ComplexityTracker() {
    clear();
}

void ComplexityTracker::clear() {
    _knownObjects.clear();
    clearQueue();
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
            _releaseExpiry = now + SETTLE_PERIOD;
        } else if (_state == ComplexityTracker::Release) {
            ++_numSlowSteps;
            if (_numSlowSteps > MAX_NUM_SLOW_STEPS) {
                _state = ComplexityTracker::Clamp;
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
            if (_quarantine.isEmpty()) {
                _state = ComplexityTracker::Inactive;
                _numSlowSteps = 0;
            } else if (_numSlowSteps > 0) {
                --_numSlowSteps;
            }
        } else {
            if (_numSlowSteps > 0) {
                --_numSlowSteps;
            } else if (_releaseExpiry < now) {
                _state = ComplexityTracker::Release;
                _numSlowSteps = 0;
            }
        }
    }

    const int32_t STEPS_BETWEEN_CHANGES = 5;
    if ((_updateCounter % STEPS_BETWEEN_CHANGES) == 0) {
        // we only allow quarantine transitions every third step
        // which gives the simulation two steps between to rebuild contact manifolds and resolve penetrations
        if (_state == ComplexityTracker::Clamp && _simulationStepRatio > FAST_RATIO) {
            const float CLAMP_RATIO = 0.11f;
            int32_t target = (int32_t)(CLAMP_RATIO * (float)_totalQueueComplexity);
          	int32_t amount = 0;
          	while(amount < target && _totalQueueComplexity > 0) {
              	Complexity complexity = popTop();
                if (complexity.value > 0) {
              	    _quarantine.insert(complexity);
              	    complexity.key->addDirtyFlags(Simulation::DIRTY_MOTION_TYPE);
              	    btRigidBody* body = complexity.key->getRigidBody();
              	    body->setCollisionFlags(body->getCollisionFlags() | CF_QUARANTINE);
              	    changedObjects.push_back(complexity.key);
              	    amount += complexity.value;
                }
          	}
            if (amount > 0) {
                _releaseExpiry = now + SETTLE_PERIOD;
            }
        } else if (_state == ComplexityTracker::Release && _simulationStepRatio < FAST_RATIO) {
            int32_t target = 1;
          	int32_t amount = 0;
          	while(amount < target && !_quarantine.isEmpty()) {
              	Complexity complexity = _quarantine.popBottom();
              	complexity.key->addDirtyFlags(Simulation::DIRTY_MOTION_TYPE);
              	btRigidBody* body = complexity.key->getRigidBody();
              	body->setCollisionFlags(body->getCollisionFlags() & ~CF_QUARANTINE);
              	changedObjects.push_back(complexity.key);
              	amount += complexity.value;
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

void ComplexityTracker::remember(ObjectMotionState* key, int32_t value) {
    ComplexityMap::iterator itr = _knownObjects.find(key);
    _totalComplexity += value;
    if (itr == _knownObjects.end()) {
        _knownObjects.insert({key, value});
    } else {
        itr->second += value;
    }
    if (! (key->getRigidBody()->getCollisionFlags() & (btCollisionObject::CF_STATIC_OBJECT | CF_QUARANTINE)) ) {
        _queueIsDirty = true;
        _totalQueueComplexity += value;
    }
}

void ComplexityTracker::forget(ObjectMotionState* key, int32_t value) {
    ComplexityMap::iterator itr = _knownObjects.find(key);
    if (itr != _knownObjects.end()) {
        _totalComplexity -= value;
        if (itr->second > value) {
            itr->second -= value;
        } else {
            _knownObjects.erase(itr);
        }
        if (! (key->getRigidBody()->getCollisionFlags() & (btCollisionObject::CF_STATIC_OBJECT | CF_QUARANTINE)) ) {
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
        if (! (itr->first->getRigidBody()->getCollisionFlags() & (btCollisionObject::CF_STATIC_OBJECT | CF_QUARANTINE)) ) {
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
        if (! (body->getCollisionFlags() & (btCollisionObject::CF_STATIC_OBJECT | CF_QUARANTINE)) ) {
            _quarantineQueue.push(Complexity({itr->first, itr->second}));
            _totalQueueComplexity += itr->second;
        }
        ++itr;
    }
    _queueIsDirty = false;
}

