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
#include <iostream> // adebug

#include "ObjectMotionState.h"

ComplexityTracker::ComplexityTracker() : _state(ComplexityTracker::Inactive) {
}

ComplexityTracker::~ComplexityTracker() {
    clear();
}

void ComplexityTracker::clear() {
    _map.clear();
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

    TrackerState oldState = _state; // adebug

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
                std::cout << "adebug 000 statechange " << oldState << " --> " << _state << std::endl;  // adebug
            }
        } else if (_state == ComplexityTracker::Ready) {
            _state = ComplexityTracker::Clamp;
            _releaseExpiry = now + SETTLE_PERIOD;
            std::cout << "adebug 001 statechange " << oldState << " --> " << _state << std::endl;  // adebug
        } else if (_state == ComplexityTracker::Release) {
            ++_numSlowSteps;
            if (_numSlowSteps > MAX_NUM_SLOW_STEPS) {
                _state = ComplexityTracker::Clamp;
                _releaseExpiry = now + SETTLE_PERIOD;
                std::cout << "adebug 002 statechange " << oldState << " --> " << _state << std::endl;  // adebug
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
            if (_queue.empty()) { // adebug TODO: figure out if this is the right test here
                _state = ComplexityTracker::Inactive;
                _numSlowSteps = 0;
                std::cout << "adebug 003 statechange " << oldState << " --> " << _state << std::endl;  // adebug
            } else if (_numSlowSteps > 0) {
                --_numSlowSteps;
            }
        } else {
            if (_releaseExpiry < now) {
                _state = ComplexityTracker::Release;
                _numSlowSteps = 0;
                std::cout << "adebug 004 statechange " << oldState << " --> " << _state << std::endl;  // adebug
            } else if (_numSlowSteps > 0) {
                --_numSlowSteps;
            }
        }
    } else {
        // between slow and fast but when actively clamping or release we maintain or drift to clamping
        if (_state == ComplexityTracker::Release) {
            ++_numSlowSteps;
            if (_numSlowSteps > MAX_NUM_SLOW_STEPS) {
                _state = ComplexityTracker::Clamp;
                _releaseExpiry = now;
                std::cout << "adebug 005 statechange " << oldState << " --> " << _state << std::endl;  // adebug
            }
        } else if (_state == ComplexityTracker::Clamp) {
            _releaseExpiry = now;
            if (_numSlowSteps < MAX_NUM_SLOW_STEPS) {
                ++_numSlowSteps;
            }
        }
    }

    if ((_updateCounter % 3) == 0) {
        // we only allow quarantine transitions every third step
        // which gives the simulation two steps between to rebuild contact manifolds and resolve penetrations
        const float CLAMP_RATIO = 0.11f;
        const float RELEASE_RATIO = (1.0f - 1.0f/CLAMP_RATIO);
        if (_state == ComplexityTracker::Clamp) {
            int32_t target = (int32_t)(CLAMP_RATIO * (float)_totalQueueComplexity);
          	int32_t amount = 0;
          	while(amount < target && _totalQueueComplexity > 0) {
              	Complexity complexity = popTop();
                // adebug TODO: make sure complexity is valid
              	_quarantine.insert(complexity);
              	complexity.key->addDirtyFlags(Simulation::DIRTY_MOTION_TYPE);
              	btRigidBody* body = complexity.key->getRigidBody();
              	body->setCollisionFlags(body->getCollisionFlags() | CF_QUARANTINE);
              	changedObjects.push_back(complexity.key);
              	amount += complexity.value;
                std::cout << "adebug + " << (void*)(complexity.key) << "  " << complexity.value << std::endl;  // adebug
          	}
        } else if (_state == ComplexityTracker::Release) {
          	int32_t target = (int32_t)(RELEASE_RATIO * (float)_quarantine.getTotalComplexity());
          	int32_t amount = 0;
          	while(amount < target && !_quarantine.isEmpty()) {
              	Complexity complexity = _quarantine.popBottom();
                // adebug TODO: convince yourself that complexity is always valid
              	complexity.key->addDirtyFlags(Simulation::DIRTY_MOTION_TYPE);
              	btRigidBody* body = complexity.key->getRigidBody();
              	body->setCollisionFlags(body->getCollisionFlags() & ~CF_QUARANTINE);
              	changedObjects.push_back(complexity.key);
              	amount += complexity.value;
                std::cout << "adebug - " << (void*)(complexity.key) << "  " << complexity.value << std::endl;  // adebug
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
    std::cout << "adebug 000 statechange " << oldState << " --> " << _state << std::endl;  // adebug
}

void ComplexityTracker::remember(ObjectMotionState* key, int32_t value) {
    // adebug TODO: assert we're in the right state, and the totals are reset when leaving ACTIVE states
    ComplexityMap::iterator itr = _map.find(key);
    _totalComplexity += value;
    if (itr == _map.end()) {
        _map.insert({key, value});
    } else {
        itr->second += value;
    }
    if (! (key->getRigidBody()->getCollisionFlags() & (btCollisionObject::CF_STATIC_OBJECT | CF_QUARANTINE)) ) {
        _queueIsDirty = true;
        _totalQueueComplexity += value;
    }
}

void ComplexityTracker::forget(ObjectMotionState* key, int32_t value) {
    // adebug TODO: assert we're in the right state
    ComplexityMap::iterator itr = _map.find(key);
    if (itr != _map.end()) {
        _totalComplexity -= value;
        if (itr->second > value) {
            itr->second -= value;
        } else {
            _map.erase(itr);
        }
        if (! (key->getRigidBody()->getCollisionFlags() & (btCollisionObject::CF_STATIC_OBJECT | CF_QUARANTINE)) ) {
            _queueIsDirty = true;
            _totalQueueComplexity -= value;
        }

        if (_map.empty()) {
            #ifdef DEBUG
            assert(_totalComplexity == 0);
            #else // DEBUG
            _totalComplexity = 0;
            #endif // DEBUG
        }
    }
}

void ComplexityTracker::remove(ObjectMotionState* key) {
    ComplexityMap::iterator itr = _map.find(key);
    if (itr != _map.end()) {
        _totalComplexity -= itr->second;
        if (! (itr->first->getRigidBody()->getCollisionFlags() & (btCollisionObject::CF_STATIC_OBJECT | CF_QUARANTINE)) ) {
            _totalQueueComplexity -= itr->second;
            _queueIsDirty = true;
        }
        _map.erase(itr);

        if (_map.empty()) {
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
    if (_map.empty()) {
        assert(false); // adebug
        return Complexity();
    }

    if (_queueIsDirty) {
        rebuildQueue();
    }

    // pop from _queue
    Complexity complexity = _queue.top();
    _totalQueueComplexity -= complexity.value;
    _queue.pop();
    return complexity;
}

void ComplexityTracker::dump() {
    std::cout << "adebug totalComplexity = " << _totalComplexity << std::endl;  // adebug
    ComplexityMap::const_iterator itr = _map.begin();
    while (itr != _map.end()) {
        std::cout << "adebug     " << (void*)(itr->first) << "  " << (itr->second) << std::endl;  // adebug
        ++itr;
    }
}

void ComplexityTracker::clearQueue() {
    while (!_queue.empty()) {
        _queue.pop();
    }
    _queueIsDirty = true;
    _totalQueueComplexity = 0;
}

void ComplexityTracker::rebuildQueue() {
    // rebuild the queue of non-static, non-quarantined objects
    clearQueue();
    ComplexityMap::const_iterator itr = _map.begin();
    while (itr != _map.end()) {
        btRigidBody* body = itr->first->getRigidBody();
        if (! (body->getCollisionFlags() & (btCollisionObject::CF_STATIC_OBJECT | CF_QUARANTINE)) ) {
            _queue.push(Complexity({itr->first, itr->second}));
            _totalQueueComplexity += itr->second;
        }
        ++itr;
    }
    _queueIsDirty = false;
}

