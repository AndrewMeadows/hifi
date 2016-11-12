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

void ComplexityTracker::remember(ObjectMotionState* state, int32_t complexity) {
    if (state->getMotionType() != MOTION_TYPE_STATIC) {
        // add this one to the map
        ComplexityMap::iterator itr = _map.find(state);
        if (itr == _map.end()) {
            _map.insert({state, complexity});
        } else {
            itr->second += complexity;
        }
        _totalComplexity += complexity;
        _queueIsDirty = true;
    }
}

void ComplexityTracker::forget(ObjectMotionState* state) {
    // remove this one from the map
    ComplexityMap::iterator itr = _map.find(state);
    if (itr != _map.end()) {
        _totalComplexity -= itr->second;
        _map.erase(itr);
        _queueIsDirty = true;

        #ifdef DEBUG
        if (_map.empty()) {
            assert(_totalComplexity == 0);
        }
        #endif // DEBUG
    }
}

Complexity ComplexityTracker::popTop() {
    if (_map.empty()) {
        return Complexity();
    }

    if (_queueIsDirty) {
        // rebuild the release queue
        clearQueue();
        ComplexityMap::const_iterator itr = _map.begin();
        while (itr != _map.end()) {
            _queue.push(Complexity({itr->first, itr->second}));
            ++itr;
        }
        _queueIsDirty = false;
    }

    // pop from _queue and remove from _map
    Complexity complexity = _queue.top();
    _queue.pop();
    _map.erase(complexity.key);
    _totalComplexity -= complexity.value;

    #ifdef DEBUG
    if (_map.empty()) {
        assert(_totalComplexity == 0);
    }
    #endif // DEBUG
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
}

