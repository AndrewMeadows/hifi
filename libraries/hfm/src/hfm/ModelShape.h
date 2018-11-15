//
//  ModelShape.h
//  libraries/hfm/src/hfm
//
//  Created by Andrew Meadows 2018/11/08.
//  Copyright 2018 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_ModelShape_h_
#define hifi_ModelShape_h_

#include "gjk.h"

#include <vector>

#include "HFM.h"

namespace gjk {

// gjk::ModelShape references external mesh data in a Model and assumes all MeshParts are convex
class ModelShape : public Shape {
public:
    ModelShape() = delete;
    ModelShape(const HFMModel* model);
    ~ModelShape();

    glm::vec3 getSupportVertex(const glm::vec3& direction) const override;
    bool containsPoint(const glm::vec3& point) const override;

    class Part : Shape {
    public:
        Part() { }
        Part(const HFMMesh* mesh, std::vector<uint32_t>& indices, const glm::mat4& invModelOffset);
        ~Part();

        glm::vec3 getSupportVertex(const glm::vec3& direction) const override;
        bool containsPoint(const glm::vec3& point) const override;

        const std::vector<uint32_t>& getIndices() const { return _indices; }
        void computeCentroid();
    private:
        std::vector<uint32_t> _indices;
        glm::mat4 _modelToMeshMatrix;
        const HFMMesh* _mesh { nullptr };
    };

protected:
    void assembleParts(const HFMModel* model);

private:
    std::vector<Part> _parts;
};

} // namespace gjk

#endif // hifi_ModelShape_h_
