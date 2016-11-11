//
//  Quarantine.cpp
//  libraries/physcis/src
//
//  Created by Andrew Meadows 2016.12.10
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "Quarantine.h"

#include "ObjectMotionState.h"

Quarantine::Quarantine() {
}

Quarantine::~Quarantine() {
    clear();
}

void Quarantine::clear() {
    _map.clear();
    _totalComplexity = 0;
}

bool Quarantine::isEmpty() const {
    return _map.empty();
}

bool Quarantine::contains(ObjectMotionState* object) const {
    return _map.find(object) != _map.end();
}

void Quarantine::isolate(ComplexityTracker& complexityTracker, float percent) {
}

void Quarantine::release(float percent) {
}

void Quarantine::release(ObjectMotionState* object) {
    ComplexityMap::iterator itr = _map.find(object);
    if (itr != _map.end()) {
        _totalComplexity -= itr->second;
        releaseObject(itr->first);
        _map.erase(itr);
        if (_map.empty()) {
            _totalComplexity = 0;
        }
    }
}

void Quarantine::dump() const {
}

void Quarantine::isolateObject(ObjectMotionState* object) {
}

void Quarantine::releaseObject(ObjectMotionState* object) {
}
