//
//  ComplexityQueue.h
//  libraries/physcis/src
//
//  Created by Andrew Meadows 2016.12.10
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_ComplexityQueue_h
#define hifi_ComplexityQueue_h

#include <queue>

class ObjectMotionState;

class Complexity {
public:
    class GreaterThan {
    public:
        bool operator() (const Complexity& A, const Complexity& B) { return A.value > B.value; }
    };

    class LessThan {
    public:
        bool operator() (const Complexity& A, const Complexity& B) { return A.value < B.value; }
    };

    ObjectMotionState* key;
    int32_t value;
};

using ComplexityQueueHighToLow = std::priority_queue<Complexity, std::vector<Complexity>, Complexity::LessThan>;
using ComplexityQueueLowToHigh = std::priority_queue<Complexity, std::vector<Complexity>, Complexity::GreaterThan>;

#endif // hifi_ComplexityQueue_h
