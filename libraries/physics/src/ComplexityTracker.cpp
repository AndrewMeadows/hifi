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
#include <iostream> // adebug

#include "ObjectMotionState.h"

ComplexityTracker::ComplexityTracker() {
}

ComplexityTracker::~ComplexityTracker() {
    clear();
}

void ComplexityTracker::clear() {
    _map.clear();
    _totalComplexity = 0;
}

void ComplexityTracker::addComplexity(ObjectMotionState* state, int32_t complexity) {
    if (state->getMotionType() != MOTION_TYPE_STATIC) {
        ComplexityMap::iterator itr = _map.find(state);
        if (itr == _map.end()) {
            _map.insert({state, complexity});
        } else {
            itr->second += complexity;
        }
        _totalComplexity += complexity;
    }
}

void ComplexityTracker::removeComplexity(ObjectMotionState* state, int32_t complexity) {
    ComplexityMap::iterator itr = _map.find(state);
    if (itr != _map.end()) {
        if (itr->second > complexity) {
            itr->second -= complexity;
            _totalComplexity -= complexity;
        } else {
            _totalComplexity -= itr->second;
            _map.erase(itr);
            if (_map.empty()) {
                _totalComplexity = 0;
            }
        }
    }
}

void ComplexityTracker::remove(ObjectMotionState* state) {
    ComplexityMap::iterator itr = _map.find(state);
    if (itr != _map.end()) {
        _totalComplexity -= itr->second;
        _map.erase(itr);
        if (_map.empty()) {
            _totalComplexity = 0;
        }
    }
}

void ComplexityTracker::dump() {
    std::cout << "adebug totalComplexity = " << _totalComplexity << std::endl;  // adebug
    ComplexityMap::const_iterator itr = _map.begin();
    while (itr != _map.end()) {
        std::cout << "adebug     " << (void*)(itr->first) << "  " << (itr->second) << std::endl;  // adebug
        ++itr;
    }
}

