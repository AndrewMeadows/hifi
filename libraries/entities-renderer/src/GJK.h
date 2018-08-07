//
//  GJK.h
//  libraries/physics/src
//
//  Created by Andrew Meadows 2018.08.03
//  Copyright 2018 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_GJK_h
#define hifi_GJK_h

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

namespace gjk {

class ConvexHull {
 public:
    ConvexHull() { }
    ConvexHull(const std::vector<glm::vec3>& localVertices);

    void setVertices(const std::vector<glm::vec3>& localVertices);
    void setWorldTransform(const glm::vec3& translation, const glm::quat& rotation);

    glm::vec3 getCentroid() const;
    glm::vec3 getSupportVertex(const glm::vec3& direction) const;
    bool containsPoint(const glm::vec3& point) const;

    // these getVertex() methods are for debug/test purposes only
    glm::vec3 getVertexLocalFrame(uint32_t index) const { return _vertices[index]; } // debug/test
    glm::vec3 getVertex(uint32_t index) const { return _translation + _rotation * _vertices[index]; } // debug/test

protected:
    void computeCentroid();

private:
    std::vector<glm::vec3> _vertices;
    glm::quat _rotation { 1.0f, 0.0f, 0.0f, 0.0f };
    glm::vec3 _translation { 0.0f, 0.0f, 0.0f };
};


bool intersect(const ConvexHull& hullA, const ConvexHull& hullB);

} // namespace gjk

#endif // hifi_GJK_h
