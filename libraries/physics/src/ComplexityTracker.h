//
//  ComplexityTracker.h
//  libraries/physcis/src
//
//  Created by Andrew Meadows 2016.11.10
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_ComplexityTracker_h
#define hifi_ComplexityTracker_h

#include <unordered_map>

class ObjectMotionState;
using ComplexityMap = std::unordered_map<ObjectMotionState*, int32_t>;

class ComplexityTracker {
public:
    ComplexityTracker();
    ~ComplexityTracker();

    void clear();

    void remember(ObjectMotionState* state, int32_t complexity);
    void forget(ObjectMotionState* state);

    void dump();

private:
    ComplexityMap _map;
    int32_t _totalComplexity { 0 };
};

#endif // hifi_ComplexityTracker_h
