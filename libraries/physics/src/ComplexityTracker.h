//
//  ComplexityTracker.h
//  libraries/physics/src
//
//  Created by Andrew Meadows 2016.11.10
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_ComplexityTracker_h
#define hifi_ComplexityTracker_h

#include "ComplexityQueue.h"
#include "ObjectMotionState.h"
#include "Quarantine.h"

class ComplexityTracker {
public:
    enum TrackerState {
        Inactive,
        EnabledButNotReady,
        Ready,
        Clamp,
        Release
    };

    ComplexityTracker();
    ~ComplexityTracker();

    void clear();

    void setSimulationStepRatio(float ratio);
    void update(VectorOfMotionStates& objects);

    bool isActive() const { return _state != Inactive; }
    void setInitialized();
    bool needsInitialization() const { return _state == ComplexityTracker::EnabledButNotReady; }

    void remember(ObjectMotionState* key, int32_t value);
    void forget(ObjectMotionState* key, int32_t value);
    void remove(ObjectMotionState* key);

    void dump();

protected:
    Complexity popTop();
    void clearQueue();
    void rebuildQueue();

private:
    Quarantine _quarantine;
    ComplexityMap _map;
    ComplexityQueueHighToLow _queue;
    uint64_t _releaseExpiry { 0 };
    int32_t _totalQueueComplexity { 0 };
    int32_t _totalComplexity { 0 };
    int32_t _numSlowSteps { 0 };
    int32_t _updateCounter { 0 };
    float _simulationStepRatio { 0.0f };
    TrackerState _state { Inactive };
    bool _queueIsDirty { true };
};

#endif // hifi_ComplexityTracker_h
