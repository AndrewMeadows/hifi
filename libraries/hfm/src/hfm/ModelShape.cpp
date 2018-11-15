//
//  ModelShape.cpp
//  libraries/hfm/src/hfm
//
//  Created by Andrew Meadows 2018/11/08.
//  Copyright 2018 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "ModelShape.h"

namespace gjk {

const std::vector<glm::vec3> CANONICAL_DIRECTIONS = {
    { 1.0f, 1.0f, 1.0f },
    { 1.0f, 1.0f, -1.0f },
    { 1.0f, -1.0f, 1.0f },
    { 1.0f, -1.0f, -1.0f },
    { -1.0f, 1.0f, 1.0f },
    { -1.0f, 1.0f, -1.0f },
    { -1.0f, -1.0f, 1.0f },
    { -1.0f, -1.0f, -1.0f },
    { 1.0f, 0.0f, 0.0f },
    { -1.0f, 0.0f, 0.0f },
    { 0.0f, 1.0f, 0.0f },
    { 0.0f, -1.0f, 0.0f },
    { 0.0f, 0.0f, 1.0f },
    { 0.0f, 0.0f, -1.0f }
};

// helper function for ModelShape::assembleParts()
void findHullIndices(const HFMMesh& mesh, const HFMMeshPart& part, std::vector<uint32_t>& indices) {
    // collect unique indices
    for (uint32_t i = 0; i < (uint32_t)part.quadIndices.size(); ++i) {
        uint32_t j = part.quadIndices[i];
        if (std::find(indices.begin(), indices.end(), j) == indices.end()) {
            indices.push_back(j);
        }
    }
    for (uint32_t i = 0; i < (uint32_t)part.quadTrianglesIndices.size(); ++i) {
        uint32_t j = part.quadTrianglesIndices[i];
        if (std::find(indices.begin(), indices.end(), j) == indices.end()) {
            indices.push_back(j);
        }
    }
    for (uint32_t i = 0; i < (uint32_t)part.triangleIndices.size(); ++i) {
        uint32_t j = part.triangleIndices[i];
        if (std::find(indices.begin(), indices.end(), j) == indices.end()) {
            indices.push_back(j);
        }
    }

    if (indices.size() < 2) {
        return;
    }

    // indices might be too big or might contain dupe points, so we collapse the collection and de-dupify
    // by only keeping the max projections along canonical directions
    std::vector<uint32_t> reducedIndices;
    for (const auto& direction : CANONICAL_DIRECTIONS) {
        float maxDot = - FLT_MAX;
        uint32_t maxIndex = 0;
        for (auto index : indices) {
            float d = glm::dot(direction, mesh.vertices[index]);
            if (d > maxDot) {
                maxDot = d;
                maxIndex = index;
            }
        }
        if (std::find(reducedIndices.begin(), reducedIndices.end(), maxIndex) == reducedIndices.end()) {
            reducedIndices.push_back(maxIndex);
        }
    }
    indices = reducedIndices;

    // sort indices in ascending order
    std::sort(indices.begin(), indices.end());
}

ModelShape::ModelShape(const HFMModel* model) {
    assert(model);
    assembleParts(model);
}

ModelShape::~ModelShape() {
    _parts.clear();
}

bool ModelShape::containsPoint(const glm::vec3& point) const {
    // Note: point is in model-frame
    for (auto& part : _parts) {
        if (part.containsPoint(point)) {
            return true;
        }
    }
    return false;
}

glm::vec3 ModelShape::getSupportVertex(const glm::vec3& direction) const {
    // this method not supported
    return glm::vec3(0.0f);
}

void ModelShape::assembleParts(const HFMModel* model) {
    assert(model);
    glm::mat4 invModelOffset = glm::inverse(model->offset);
    for (uint32_t i = 0; i < (uint32_t)(model->meshes.size()); ++i) {
        const HFMMesh& mesh = model->meshes[i];
        for (const auto& part : mesh.parts) {
            std::vector<uint32_t> indices;
            findHullIndices(mesh, part, indices);
            if (indices.size() > 0) {
                _parts.push_back(std::move(ModelShape::Part(&mesh, indices, invModelOffset)));
            }
        }
    }
    for (auto& part : _parts) {
        part.computeCentroid();
    }
}

ModelShape::Part::Part(const HFMMesh* mesh, std::vector<uint32_t>& indices, const glm::mat4& invModelOffset)
: _mesh(mesh) {
    assert(_mesh);
    _indices = std::move(indices);
    _modelToMeshMatrix = glm::inverse(_mesh->modelTransform) * invModelOffset;
};

glm::vec3 ModelShape::Part::getSupportVertex(const glm::vec3& direction) const {
    const auto& vertices = _mesh->vertices;
    uint32_t maxIndex = _indices[0];
    float maxDot = glm::dot(direction, vertices[maxIndex]);
    for (uint32_t i = 1; i < _indices.size(); ++i) {
        uint32_t index = _indices[i];
        float d = glm::dot(direction, vertices[index]);
        if (d > maxDot) {
            maxDot = d;
            maxIndex = index;
        }
    }
    return vertices[maxIndex];
}

ModelShape::Part::~Part() {
    _mesh = nullptr;
}

bool ModelShape::Part::containsPoint(const glm::vec3& modelPoint) const {
    glm::vec3 meshPoint = glm::vec3(_modelToMeshMatrix * glm::vec4(modelPoint, 1.0f));
    SphereShape sphere(meshPoint, 0.0f);
    Transform identity;
    identity.setIdentity();
    return intersect(identity, *this, identity, sphere);
}

void ModelShape::Part::computeCentroid() {
    const auto& vertices = _mesh->vertices;
    glm::vec3 accumulator(0.0f);
    for (uint32_t i = 0; i < _indices.size(); ++i) {
        accumulator += vertices[_indices[i]];
    }
    _centroid = accumulator / (float)_indices.size();
}

} // namespace gjk
