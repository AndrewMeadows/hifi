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

#include "gjk.h"

#include <glm/gtx/norm.hpp>

using namespace gjk;

void Shape::setWorldTransform(const glm::vec3& translation, const glm::quat& rotation) {
    _rotation = rotation;
    _translation = translation;
}

ConvexHull::ConvexHull(const std::vector<glm::vec3>& localVertices) {
    assert(!localVertices.empty());
    _vertices = localVertices;
    computeCentroid();
}

void ConvexHull::setVertices(const std::vector<glm::vec3>& localVertices) {
    _vertices = localVertices;
    computeCentroid();
}

glm::vec3 ConvexHull::getCentroid() const {
    // centroid is cached at the end of _vertices
    return _translation + _rotation * _vertices[_vertices.size() - 1]; // world-frame
}

glm::vec3 ConvexHull::getSupportVertex(const glm::vec3& direction) const {
    // transform direction into local frame
    glm::vec3 dir = glm::inverse(_rotation) * direction;

    float maxDot = glm::dot(dir, _vertices[0]);
    float j = 0;
    uint32_t numVertices = _vertices.size() - 1; // avoid centroid at end
    for (uint32_t i = 1; i < numVertices; ++i) {
        float dot = glm::dot(dir, _vertices[i]);
        if (dot > maxDot) {
            j = i;
            maxDot = dot;
        }
    }
    // transform result into world-frame
    return _translation + _rotation * _vertices[j];
}

bool ConvexHull::containsPoint(const glm::vec3& point) const {
    std::vector<glm::vec3> vertices;
    vertices.push_back(point);
    ConvexHull pointHull(vertices);
    return intersect(*this, pointHull);
}

void ConvexHull::computeCentroid() {
    // compute average point and cache at the end of _vertices
    uint32_t numVerts = _vertices.size();
    if (numVerts > 0) {
        glm::vec3 centroid = _vertices[0];
        for (uint32_t i = 1; i < numVerts; ++i) {
            centroid += _vertices[i];
        }
        _vertices.push_back(centroid / (float)(numVerts));
    }
}

bool evolveSimplex(std::vector<glm::vec3>& simplex, glm::vec3& direction, const Shape& shapeA, const Shape& shapeB) {
    // The most recent addition to the simplex is always the last element and we call this point 'A'.
    glm::vec3 A = simplex.back();

    // Each new point on the simplex must extend past the origin along the search direction,
    // else the two shapes cannot possibly overlap.
    if (glm::dot(direction, A) < 0.0f) {
        // latest point on simplex is not beyond origin along direction
        // we clear simplex to signal NO INTERSECTION
        simplex.clear();
        // and return false to end while-looped search
        return false;
    }

    // we assume we already have at least two points in the simplex at this point
    uint32_t n = simplex.size();
    switch (n) {
        case 2: {
            // the simplex is a line [BA]
            glm::vec3 AB = simplex[0] - A;
            if (glm::dot(AB, -A) < 0.0f) {
                // A is the best simplex
                simplex[0] = A;
                // search toward the origin
                direction = -A;
                simplex.pop_back();
            } else {
                // the line is the best simplex
                // search perpendicular to line, toward the origin
                direction = glm::cross(glm::cross(A, AB), AB);
            }
            if (glm::length2(direction) == 0.0f) {
                // origin is on line
                return false;
            }
            simplex.push_back(shapeA.getSupportVertex(direction) - shapeB.getSupportVertex(-direction));
        }
        break;
        case 3: {
            // the simplex is a triangle [CBA]
            // the triangleNormal is defined using the right-hand rule, winding A to B to C
            glm::vec3 triangleNormal = glm::cross(simplex[1] - A, simplex[0] - A);
            if (glm::length2(triangleNormal) == 0.0f) {
                // origin is in the plane
                return false;
            }
            if (glm::dot(triangleNormal, -A) < 0.0f) {
                // the triangleNormal points away from the origin so we reverse triangle winding
                // before searching for the next point
                triangleNormal = -triangleNormal;
                direction = simplex[0];
                simplex[0] = simplex[1];
                simplex[1] = direction;
            }

            // search along triangleNormal
            direction = triangleNormal;
            A = shapeA.getSupportVertex(direction) - shapeB.getSupportVertex(-direction);
            simplex.push_back(A);
        }
        break;
        case 4: {
            // simplex is a tetrahedron
            // origin and 'A' are guaranteed to be on positive side of right-hand triangle [DCB]
            // so we need to check each new face check to see if origin is in front or behind
            uint32_t i = 0;
            for (i = 0; i < 3; ++i) {
                uint32_t j = (i + 1) % 3;
                glm::vec3 B = simplex[j];
                glm::vec3 C = simplex[i];
                glm::vec3 triangleCenter = A + B + C;
                glm::vec3 triangleNormal = glm::cross(B - A, C - A);
                float thisDot = glm::dot(triangleNormal, -triangleCenter);
                if (thisDot < 0.0f) {
                    // origin is behind face
                    continue;
                }
                // origin is in front of face, so we reduce the simplex to this face
                // (with correct winding, of course) and search again
                simplex[0] = C;
                simplex[1] = B;
                simplex[2] = A;
                simplex.pop_back();

                // searh along triangleNormal
                direction = triangleNormal;
                A = shapeA.getSupportVertex(direction) - shapeB.getSupportVertex(-direction);
                simplex.push_back(A);
                break;
            }
            if (i == 3) {
                // origin is inside tetrahedron
                return false;
            }
        }
        break;
        default: {
            // should never fall in here
            assert(false);
            simplex.clear();
            return false;
        }
        break;
    }
    return true;
}

bool gjk::intersect(const Shape& shapeA, const Shape& shapeB) {
    // The GJK algorithm works as follows:
    //
    // When two convex shapes overlap: the surface of their Minkowski sum contains the origin.
    // So the problem becomes: How to find a sub-volume of the Minkowski sum that contains the origin?
    // If such a sub-volume exists then the shapes overlap, otherwise they don't.
    //
    // The 'simplex' is a list of points on the Minkowski surface that define the sub-volume.
    // It starts with a single point.  We search toward the origin for a new point on the Minkowski surface and
    // check to see if the origin is inside the sub-volume.  We continue to grow the simplex, starting with
    // a single point to make a line-segment (2 points), then a triangle (3), and finally a tetrahedron (4).
    // Each new point is obtained by searching the Minkowski surface toward the origin, and each time we get
    // a new simplex we check to see if we can prove the origin is inside or outside.

    glm::vec3 dir = shapeA.getCentroid() - shapeB.getCentroid();
    const float MIN_CENTROID_SEPARATION_SQUARED = 1.0e-10f;
    float distance2 = glm::length2(dir);
    if (distance2 < MIN_CENTROID_SEPARATION_SQUARED) {
        return true;
    }

    glm::vec3 B = shapeA.getSupportVertex(dir) - shapeB.getSupportVertex(-dir);
    if (glm::dot(dir, B) < 0.0f) {
        // when the new point on the Minkowski surface is not on the other side of the origin
        // along the direction of the search --> we immediately know the two shapes cannot possibly overlap
        return false;
    }
    dir = -dir;
    glm::vec3 A = shapeA.getSupportVertex(dir) - shapeB.getSupportVertex(-dir);
    if (glm::dot(dir, A) < 0.0f) {
        return false;
    }

    std::vector<glm::vec3> simplex;
    simplex.push_back(B);
    simplex.push_back(A);

    const uint32_t MAX_NUM_LOOPS = 10;
    uint32_t numLoops = 0;
    while (evolveSimplex(simplex, dir, shapeA, shapeB)) {
        if (++numLoops > MAX_NUM_LOOPS) {
            // could not find origin inside Minkowski surface
            return false;
        }
    }
    // an empty simplex is used to signal NO intersection
    return !simplex.empty();
}
