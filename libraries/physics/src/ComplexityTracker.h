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


const int32_t CF_QUARANTINE_SET_STATIC = 0x01 << 30; // unused by Bullet
const int32_t CF_QUARANTINE_SOFTEN_COLLISIONS = 0x01 << 29; // unused by Bullet
const int32_t CF_QUARANTINE_FLAGS = CF_QUARANTINE_SET_STATIC | CF_QUARANTINE_SOFTEN_COLLISIONS;


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

    void remember(ObjectMotionState* key, int64_t value);
    void forget(ObjectMotionState* key, int64_t value);
    void remove(ObjectMotionState* key);

protected:
    Complexity popTop();
    void clearQueue();
    void rebuildQueue();

private:
    Quarantine _quarantine;
    ComplexityMap _knownObjects;
    ComplexityQueueHighToLow _quarantineQueue;
    SoftMap _softMap;
    uint64_t _releaseExpiry { 0 };
    uint64_t _nextSoftExpiry { 0 };
    int64_t _totalQueueComplexity { 0 };
    int64_t _totalComplexity { 0 };
    int32_t _numSlowSteps { 0 };
    int32_t _updateCounter { 0 };
    float _simulationStepRatio { 0.0f };
    TrackerState _state { Inactive };
    bool _queueIsDirty { true };
};

#endif // hifi_ComplexityTracker_h
