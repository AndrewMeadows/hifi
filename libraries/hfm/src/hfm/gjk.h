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

#include <Transform.h>

namespace gjk {

// The GJK algorithm is a fast way to determine whether two convex shapes overlap or not.

// gjk::Shape defines the API of convex shapes
class Shape {
public:
    Shape() { }
    virtual ~Shape() { }
    // all arguments and return values are in the local-frame
    virtual glm::vec3 getSupportVertex(const glm::vec3& direction) const = 0;
    virtual bool containsPoint(const glm::vec3& point) const = 0;
    glm::vec3 getCentroid() const { return _centroid; }
protected:
    glm::vec3 _centroid;
};

// gjk::SphereShape is the simplest of convex shapes
class SphereShape : public Shape {
public:
    SphereShape() { }
    SphereShape(const glm::vec3& center, float radius);
    glm::vec3 getSupportVertex(const glm::vec3& direction) const override;
    bool containsPoint(const glm::vec3& point) const override;
private:
    float _radius { 0.0f };
};

// gjk::ConvexHullShape is useful for testing/verifying the GJK algorithm
class ConvexHullShape : public Shape {
public:
    ConvexHullShape() { }
    ConvexHullShape(const std::vector<glm::vec3>& vertices);
    glm::vec3 getSupportVertex(const glm::vec3& direction) const override;
    bool containsPoint(const glm::vec3& point) const override;
    void setVertices(const std::vector<glm::vec3>& vertices);
    // getVertex() is for debug/test purposes only
    glm::vec3 getVertex(uint32_t index) const { return _vertices[index]; }
private:
    std::vector<glm::vec3> _vertices;
    glm::vec3 _centroid { 0.0f, 0.0f, 0.0f };
};

bool intersect(const Transform& transformA, const Shape& hullA, const Transform& transformB, const Shape& hullB);

} // namespace gjk

#endif // hifi_gjk_h
