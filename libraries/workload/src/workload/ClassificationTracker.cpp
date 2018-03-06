//
//  ClassificationTracker.cpp
//  libraries/workload/src/workload
//
//  Created by Andrew Meadows 2018.02.21
//  Copyright 2018 High Fidelity, Inc.
//
//  Originally from lighthouse3d. Modified to utilize glm::vec3 and clean up to our coding standards
//  Simple plane class.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//
#include "ClassificationTracker.h"

using namespace workload;

void ClassificationTracker::configure(const Config& config) {
}

void ClassificationTracker::run(const WorkloadContextPointer& context, Outputs& outputs) {
    auto space = context->_space;
    if (space) {
        Changes changes;
        space->categorizeAndGetChanges(changes);
        // NOTE: changes are sorted by index
        // because Space::categorizeAndGetChanges() builds changes in sequenced order
        // therefore outputs will also be sorted by index.
    #ifdef USE_TRANSITION_GRID
        // build transition grid of lists between regions
        outputs.resize(workload::Space::NUM_TRANSITIONS);
        for (uint32_t i = 0; i < changes.size(); ++i) {
            int32_t j = Space::computeTransitionIndex(changes[i].prevRegion, changes[i].region);
            assert(j >= 0 && j < workload::Space::NUM_TRANSITIONS);
            outputs[j].push_back(changes[i].proxyId);
        }
    #else
        // use exit/enter lists per region
        outputs.resize(2 * workload::Space::NUM_CLASSIFICATIONS);
        for (uint32_t i = 0; i < changes.size(); ++i) {
            Space::Change& change = changes[i];
            if (change.prevRegion != Space::REGION_INVALID) {
                // EXIT list index = 2 * regionIndex
                outputs[2 * change.prevRegion].push_back(change.proxyId);
            }
            if (change.region != Space::REGION_INVALID) {
                // ENTER list index = 2 * regionIndex + 1
                outputs[2 * change.region + 1].push_back(change.proxyId);
            }
        }
    #endif
    }
}

