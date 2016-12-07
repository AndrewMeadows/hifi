//
//  Quarantine.h
//  libraries/physics/src
//
//  Created by Andrew Meadows 2016.12.10
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_Quarantine_h
#define hifi_Quarantine_h

#include <unordered_map>

#include "ComplexityQueue.h"

class ObjectMotionState;

using ComplexityMap = std::unordered_map<ObjectMotionState*, int64_t>;
using SoftMap = std::unordered_map<ObjectMotionState*, uint64_t>;

class Quarantine {
public:
    Quarantine();
    ~Quarantine();

    void clear();

    bool isEmpty() const;
    int32_t size() const;

    bool contains(ObjectMotionState* object) const;

    Complexity bottom();
    Complexity pop();
    void insert(const Complexity& complexity);
    void release(ObjectMotionState* object);

    int64_t getTotalComplexity() const { return _totalComplexity; }

private:
    void clearQueue();

private:
    ComplexityMap _map;
    ComplexityQueueLowToHigh _queue;
    int64_t _totalComplexity { 0UL };
    bool _queueIsDirty { false };
};

#endif // hifi_Quarantine_h
