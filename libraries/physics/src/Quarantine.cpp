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

// helper function
void isolateObjectHelper(ObjectMotionState* object) {
    // TODO: implement this
}

// helper function
void releaseObjectHelper(ObjectMotionState* object) {
    // TODO: implement this
}

Quarantine::Quarantine() {
}

Quarantine::~Quarantine() {
    clear();
}

void Quarantine::clear() {
    _map.clear();
	clearQueue();
    _totalComplexity = 0;
    _queueIsDirty = true;
}

bool Quarantine::isEmpty() const {
    return _map.empty();
}

bool Quarantine::contains(ObjectMotionState* object) const {
    return _map.find(object) != _map.end();
}

void Quarantine::isolate(ComplexityTracker& tracker, float percent) {
    int32_t totalComplexity = tracker.getTotalComplexity();
    if (totalComplexity == 0) {
        return;
    }
    // quarantine new objects until we have enough
    int32_t isolatedComplexity = 0;
    int32_t enoughComplexity = (int32_t)(percent * (float)totalComplexity);
    while(isolatedComplexity < enoughComplexity && !tracker.isEmpty()) {
        Complexity complexity = tracker.popTop();
        ComplexityMap::const_iterator itr = _map.find(complexity.key);
        if (itr == _map.end()) {
            _map.insert({ complexity.key, complexity.value});
            isolateObjectHelper(complexity.key);
            isolatedComplexity += complexity.value;
        }
    }
    _totalComplexity += isolatedComplexity;
    _queueIsDirty = true;
}

void Quarantine::release(float percent) {
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

    // release objects from quarantine until we have enough
    int32_t releasedComplexity = 0;
    int32_t enoughComplexity = (int32_t)(percent * (float)_totalComplexity);
    while(releasedComplexity < enoughComplexity && !_map.empty()) {
        const Complexity& complexity = _queue.top();
        releasedComplexity += complexity.value;
        release(complexity.key);
        _queue.pop();
	}
    _totalComplexity -= releasedComplexity;

    #ifdef DEBUG
    if (_map.empty()) {
        assert(_totalComplexity == 0);
    }
    #endif // DEBUG
}

void Quarantine::release(ObjectMotionState* object) {
    // release a particular object out of order
    ComplexityMap::iterator itr = _map.find(object);
    if (itr != _map.end()) {
        _totalComplexity -= itr->second;
        releaseObjectHelper(itr->first);
        _map.erase(itr);
    	_queueIsDirty = true;

        #ifdef DEBUG
        if (_map.empty()) {
            assert(_totalComplexity == 0);
        }
        #endif // DEBUG
    }
}

void Quarantine::dump() const {
}

void Quarantine::clearQueue() {
    while (!_queue.empty()) {
        _queue.pop();
    }
    _queueIsDirty = true;
}
