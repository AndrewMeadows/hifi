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

#include "FBXGeometryStats.h"

#include <iostream> // adebug

//template<class T> uint32_t hashElement(const T&);

uint32_t hashElement(const glm::vec3& v) {
    uint32_t h = std::hash<float>{}(v.x);
    h |= std::hash<float>{}(v.y);
    h += std::hash<float>{}(v.z);
    return h;
}

uint32_t hashElement(const glm::vec4& v) {
    uint32_t h = std::hash<float>{}(v.x);
    h |= std::hash<float>{}(v.y);
    h += std::hash<float>{}(v.z);
    h |= std::hash<float>{}(v.w);
    return h;
}

uint32_t hashElement(const QVector<glm::vec3>& vec3s) {
    int32_t numVec3s = vec3s.size();
    uint32_t h = std::hash<int32_t>{}(numVec3s);
    for (int32_t i = 0; i < numVec3s; ++i) {
        h += hashElement(vec3s[i]);
    }
    return h;
}

uint32_t hashElement(const glm::mat4& mat) {
    uint32_t h = hashElement(mat[0]);
    h |= hashElement(mat[1]);
    h += hashElement(mat[2]);
    h |= hashElement(mat[3]);
    return h;
}

uint32_t hashInts(const QVector<int32_t>& ints) {
    int32_t numInts = ints.size();
    uint32_t h = std::hash<int32_t>{}(numInts);
    for (int32_t i = 0; i < numInts; ++i) {
        h += std::hash<int32_t>{}(ints[i]);
    }
    return h;
}

uint32_t hashCluster(const FBXCluster& cluster) {
    uint32_t h = std::hash<int32_t>{}(cluster.jointIndex);
    h |= hashElement(cluster.inverseBindMatrix);
    return h;
}

uint32_t hashMeshPart(const FBXMeshPart& part) {
    uint32_t h = hashInts(part.quadIndices);
    h |= hashInts(part.quadTrianglesIndices);
    h += hashInts(part.triangleIndices);
    return h;
}

template<class T> uint32_t hashQVector(const QVector<T>& v) {
    uint32_t h = std::hash<int32_t>{}(v.size());
    for (const T& e : v) {
        h += hashElement(e);
    }
    return h;
}

uint32_t hashMesh(const FBXMesh& mesh) {
    int32_t numParts = mesh.parts.size();
    uint32_t h = std::hash<int32_t>{}(numParts);
    for (int i = 0; i < numParts; ++i) {
        h += hashMeshPart(mesh.parts[i]);
    }

    h |= hashQVector(mesh.vertices);
    h += hashQVector(mesh.normals);
    h |= hashQVector(mesh.tangents);
    h += hashQVector(mesh.clusterIndices);
    h |= hashQVector(mesh.clusterWeights);

    int32_t numClusters = mesh.clusters.size();
    h += std::hash<int32_t>{}(numClusters);
    for (int i = 0; i < numClusters; ++i) {
        h += hashCluster(mesh.clusters[i]);
    }

    return h;
}

uint32_t FBXGeometryStats::Entry::process(const FBXGeometry& geometry) {
    numMeshes = geometry.meshes.size();
    size_t h = std::hash<int32_t>{}(numMeshes);
    totalNumClusters = 0;
    maxClustersPerMesh = 0;
    maxClustersPerJoint = 0;
    for (const auto& mesh : geometry.meshes) {
        h += hashMesh(mesh);
        uint32_t numClusters = mesh.clusters.size();
        totalNumClusters += numClusters;
        if (numClusters > maxClustersPerMesh) {
            maxClustersPerMesh = numClusters;
        }
    }
    return h;
}

void FBXGeometryStats::process(const FBXGeometry& geometry) {
    FBXGeometryStats::Entry sample;
    uint32_t h = sample.process(geometry);
    if (_samples.find(h) == _samples.end()) {
        _samples[h] = sample;
        if (sample.maxClustersPerMesh > _maxClustersPerMesh) {
            _maxClustersPerMesh = sample.maxClustersPerMesh;
        }
        _totalNumMeshes += sample.numMeshes;
        _totalNumClusters += sample.totalNumClusters;
    }
}

void FBXGeometryStats::dump() {
    if (_totalNumMeshes > 0) {
        float avgClustersPerMesh = (float)_totalNumClusters / (float)_totalNumMeshes;
        std::cout << "adebug avgClustersPerMesh = " << avgClustersPerMesh << std::endl;  // adebug
    }
    std::cout << "adebug maxClustersPerMesh = " << _maxClustersPerMesh << std::endl;  // adebug
}

