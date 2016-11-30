//
//  ComplexityTracker.cpp
//  libraries/physcis/src
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
#include "Quarantine.h"

ComplexityTracker::ComplexityTracker() {
}

ComplexityTracker::~ComplexityTracker() {
    clear();
}

void ComplexityTracker::clear() {
    _map.clear();
    clearQueue();
    _totalComplexity = 0;
}

void ComplexityTracker::enable() {
    if (!_enabled) {
        _enabled = true;
        _initialized = false;
    }
}

void ComplexityTracker::disable() {
    if (_enabled) {
        _enabled = false;
        clear();
    }
}

bool ComplexityTracker::needsInitialization() const {
    return _enabled && !_initialized;
}

void ComplexityTracker::setInitialized() {
    _initialized = true;
}

void ComplexityTracker::remember(ObjectMotionState* key, int32_t value) {
    assert(_enabled);
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
    assert(_enabled && _initialized);
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
}

Complexity ComplexityTracker::popTop() {
    if (_map.empty()) {
        return Complexity();
    }

    if (_queueIsDirty) {
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

