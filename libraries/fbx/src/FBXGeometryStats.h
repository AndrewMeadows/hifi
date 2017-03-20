//
//  FBXGeometryStats.h
//  libraries/fbx/src
//
//  Created by Andrew Meadows 2017.03.16
//  Copyright 2017 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_FBXGeometryStats_h
#define hifi_FBXGeometryStats_h

#include <functional>
#include <map>

#include <QVector>

#include "FBXReader.h"

class FBXGeometryStats {
public:
    class Entry {
    public:
        uint32_t  process(const FBXGeometry& geometry);
        uint32_t numMeshes { 0 };
        uint32_t totalNumClusters { 0 };
        uint32_t maxClustersPerMesh { 0 };
        uint32_t maxClustersPerJoint { 0 };
    };

    FBXGeometryStats() {}

    void process(const FBXGeometry& geometry);

    void dump();

private:
    std::map<int32_t, Entry> _samples;
    uint32_t _totalNumMeshes { 0 };
    uint32_t _totalNumClusters { 0 };
    uint32_t _maxClustersPerMesh { 0 };
};


#endif // hifi_FBXGeometryStats_h
