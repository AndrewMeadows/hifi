//
//  Prioritizer.h
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

#ifndef hifi_workload_Prioritizer_h
#define hifi_workload_Prioritizer_h

#include "Space.h"
#include "Engine.h"

namespace workload {

    class PrioritizerConfig : public Job::Config {
        Q_OBJECT
    public:
        PrioritizerConfig() : Job::Config(true) {}
    };

    class Prioritizer {
    public:
        using Config = PrioritizerConfig;
        using Inputs = IndexVectors;
        using Outputs = IndexVectors;
        using JobModel = workload::Job::ModelIO<Prioritizer, Inputs, Outputs, Config>;

        Prioritizer(task::Varying& foo);

        void configure(const Config& config);
        void run(const workload::WorkloadContextPointer& context, const Inputs& SortedChanges, Outputs& outputs);

    protected:
        // _state is a list of lists of proxy indices, sorted by region
        IndexVectors _state;
    };

} // namespace workload

#endif // hifi_workload_Prioritizer_h
