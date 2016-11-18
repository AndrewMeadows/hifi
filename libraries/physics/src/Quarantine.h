//
//  Quarantine.h
//  libraries/physcis/src
//
//  Created by Andrew Meadows 2016.12.10
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_Quarantine_h
#define hifi_Quarantine_h

#include "ComplexityTracker.h"

const int CF_QUARANTINE = 0x01 << 30; // unused by Bullet

class ObjectMotionState;

class Quarantine {
public:
    Quarantine();
    ~Quarantine();

    void clear();
    bool isEmpty() const;

    bool contains(ObjectMotionState* object) const;

    Complexity popBottom();
    void insert(const Complexity& complexity);
    void release(ObjectMotionState* object);

    int32_t getTotalComplexity() const { return _totalComplexity; }

    void dump() const;

private:
    void clearQueue();

private:
    ComplexityMap _map;
    ComplexityQueueLowToHigh _queue;
    int32_t _totalComplexity { 0 };
    bool _queueIsDirty { false };
};

#endif // hifi_Quarantine_h
