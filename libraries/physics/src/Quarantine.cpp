//
//  Quarantine.cpp
//  libraries/physics/src
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
	clearQueue();
    _totalComplexity = 0L;
    _queueIsDirty = true;
}

bool Quarantine::isEmpty() const {
    return _map.empty();
}

int32_t Quarantine::size() const {
    return _queue.size();
}

bool Quarantine::contains(ObjectMotionState* object) const {
    return _map.find(object) != _map.end();
}

Complexity Quarantine::bottom() {
    // NOTE: the queue is inverted: low values at the top and high values deeper in the stack,
    // which is why this method is called 'bottom'
    assert(!_map.empty());
    if (_queueIsDirty) {
        // rebuild the queue
        clearQueue();
        ComplexityMap::const_iterator itr = _map.begin();
        while (itr != _map.end()) {
            _queue.push(Complexity({itr->first, itr->second}));
            ++itr;
        }
        _queueIsDirty = false;
    }
    return _queue.top();
}

Complexity Quarantine::pop() {
    Complexity complexity = bottom();
    _queue.pop();
    ComplexityMap::const_iterator itr = _map.find(complexity.key);
    assert(itr != _map.end());
    _map.erase(itr);
    _totalComplexity -= complexity.value;
    return complexity;
}

void Quarantine::insert(const Complexity& complexity) {
    assert(complexity.key);
    ComplexityMap::iterator itr = _map.find(complexity.key);
    if (itr == _map.end()) {
        _map.insert({complexity.key, complexity.value});
        _totalComplexity += complexity.value;
    } else if (complexity.value != itr->second) {
        _totalComplexity += complexity.value - itr->second;
        itr->second = complexity.value;
    }
    _queueIsDirty = true;
}

void Quarantine::release(ObjectMotionState* object) {
    // release a particular object out of order
    ComplexityMap::iterator itr = _map.find(object);
    if (itr != _map.end()) {
        _totalComplexity -= itr->second;
        _map.erase(itr);
    	_queueIsDirty = true;

        if (_map.empty()) {
            #ifdef DEBUG
            assert(_totalComplexity == 0);
            #else // DEBUG
            _totalComplexity = 0;
            #endif // DEBUG
        }
    }
}

void Quarantine::clearQueue() {
    while (!_queue.empty()) {
        _queue.pop();
    }
    _queueIsDirty = true;
}
