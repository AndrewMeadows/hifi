//
//  gjk.h
//  libraries/physics/src
//
//  Created by Andrew Meadows 2018.08.03
//  Copyright 2018 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_gjk_h
#define hifi_gjk_h

#include <vector>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

namespace gjk {
    // The GJK algorithm is a fast way to determine whether two convex shapes overlap or not.

class Shape {
public:
    Shape() { }
    void setWorldTransform(const glm::vec3& translation, const glm::quat& rotation);
    virtual glm::vec3 getSupportVertex(const glm::vec3& direction) const = 0; // in world-frame
    virtual bool containsPoint(const glm::vec3& point) const = 0; // in world-frame
    virtual glm::vec3 getCentroid() const = 0; // in world-frame
protected:
    glm::quat _rotation { 1.0f, 0.0f, 0.0f, 0.0f };
    glm::vec3 _translation { 0.0f, 0.0f, 0.0f };
};

class ConvexHull : public Shape {
public:
    ConvexHull() { }
    ConvexHull(const std::vector<glm::vec3>& localVertices);

    void setVertices(const std::vector<glm::vec3>& localVertices);

    glm::vec3 getCentroid() const override; // in world-frame
    glm::vec3 getSupportVertex(const glm::vec3& direction) const override; // in world-frame
    bool containsPoint(const glm::vec3& point) const override; // in world-frame

    // these getVertex() methods are for debug/test purposes only
    glm::vec3 getVertex(uint32_t index) const { return _translation + _rotation * _vertices[index]; }
    glm::vec3 getVertexLocalFrame(uint32_t index) const { return _vertices[index]; }

protected:
    void computeCentroid();

private:
    std::vector<glm::vec3> _vertices;
};


bool intersect(const Shape& hullA, const Shape& hullB);

} // namespace gjk

#endif // hifi_gjk_h
