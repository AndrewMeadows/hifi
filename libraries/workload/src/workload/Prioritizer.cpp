//
//  Prioritizer.cpp
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
#include "Prioritizer.h"

using namespace workload;

Prioritizer::Prioritizer(task::Varying&) {
    // we don't bother tracking the state of REGION_UNKNOWN
    // so we reserve for state up to that region index.
    _state.reserve(Space::REGION_UNKNOWN);
}

void Prioritizer::configure(const Config& config) {
}

void Prioritizer::run(const workload::WorkloadContextPointer& context, const Inputs& inputs, Outputs& outputs) {
    if (inputs.size() == 0) {
        return;
    }
    assert(inputs.size() == 2 * Space::REGION_INVALID);

    for (uint32_t i = 0; i < _state.size(); ++i) {
        uint32_t exitIndex = 2 * i;
        uint32_t enterIndex = exitIndex + 1;
        const IndexList& going = inputs[exitIndex];
        const IndexList& coming = inputs[enterIndex];
        if (coming.size() == 0 && going.size() == 0) {
            continue;
        }

        if (_state[i].empty()) {
            assert(going.empty());
            _state[i] = coming;
        } else {
            IndexList& oldState = _state[i];
            IndexList newState;
            newState.reserve(oldState.size() - going.size() + coming.size());
            uint32_t g = 0;
            uint32_t c = 0;
            for (uint32_t j = 0; j < oldState.size(); ++j) {
                // NOTE: all lists are sorted by index!
                while (c < coming.size() && coming[c] < oldState[j]) {
                    newState.push_back(coming[c]);
                    ++c;
                }
                if (going[g] == oldState[j]) {
                    ++g;
                } else {
                    newState.push_back(oldState[j]);
                }
            }
            assert(g == going.size());
            while (c < coming.size()) {
                newState.push_back(coming[c]);
                ++c;
            }
            oldState.swap(newState);
        }
    }
}

