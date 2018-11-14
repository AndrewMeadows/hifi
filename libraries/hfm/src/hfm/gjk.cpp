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

SphereShape::SphereShape(const glm::vec3& center, float radius) : _radius(radius) {
    _centroid = center;
}

glm::vec3 SphereShape::getSupportVertex(const glm::vec3& direction) const {
    // NOTE: direction is not necessarily normalized so we must normalize it here
    float directionLength = glm::length(direction);
    const float MIN_DIRECTION_LENGTH = 1.0e-6f;
    if (directionLength < MIN_DIRECTION_LENGTH) {
        return _centroid;
    }
    return _centroid + (_radius / directionLength) * direction;
}

bool SphereShape::containsPoint(const glm::vec3& point) const {
    return glm::distance(point, _centroid) <= _radius;
}

ConvexHullShape::ConvexHullShape(const std::vector<glm::vec3>& vertices) {
    assert(!vertices.empty());
    setVertices(vertices);
}

void ConvexHullShape::setVertices(const std::vector<glm::vec3>& vertices) {
    _vertices = vertices;

    // compute centroid
    uint32_t numVerts = _vertices.size();
    if (numVerts > 0) {
        glm::vec3 accumulator = _vertices[0];
        for (uint32_t i = 1; i < numVerts; ++i) {
            accumulator += _vertices[i];
        }
        _centroid = accumulator / (float)(numVerts);
    } else {
        _centroid = glm::vec3(0.0f, 0.0f, 0.0f);
    }
}

glm::vec3 ConvexHullShape::getSupportVertex(const glm::vec3& direction) const {
    float maxDot = glm::dot(direction, _vertices[0]);
    float j = 0;
    uint32_t numVertices = _vertices.size();
    for (uint32_t i = 1; i < numVertices; ++i) {
        float dot = glm::dot(direction, _vertices[i]);
        if (dot > maxDot) {
            j = i;
            maxDot = dot;
        }
    }
    return _vertices[j];
}

bool ConvexHullShape::containsPoint(const glm::vec3& point) const {
    SphereShape sphere(point, 0.0f);
    Transform identity;
    identity.setIdentity();
    return intersect(identity, *this, identity, sphere);
}

// MinkowskiSurface is a helper class for gjk::intersect()
class MinkowskiSurface {
public:
    MinkowskiSurface() = delete;
    MinkowskiSurface(const Transform& transformA, const Shape* shapeA, const Transform& transformB, const Shape* shapeB);
    glm::vec3 getStartDirection() const;
    glm::vec3 getSupportVertex(const glm::vec3& direction) const;
private:
    Transform _transformBtoA;
    Transform _transformAtoB;
    const Shape* _shapeA;
    const Shape* _shapeB;
};

MinkowskiSurface::MinkowskiSurface(const Transform& transformA, const Shape* shapeA, const Transform& transformB, const Shape* shapeB)
: _shapeA(shapeA), _shapeB(shapeB) {
    assert(_shapeA);
    assert(_shapeB);
    // Note: to simplify the code we compute the MinkowskiSurface in shapeA's local-frame,
    // we compute _transformBtoA and _transformAtoB to help us do that
    Transform::inverseMult(_transformBtoA, transformA, transformB);
    _transformBtoA.evalInverse(_transformAtoB);
}

glm::vec3 MinkowskiSurface::getStartDirection() const {
    return _shapeA->getCentroid() - _transformBtoA.transform(_shapeB->getCentroid());
}

glm::vec3 MinkowskiSurface::getSupportVertex(const glm::vec3& direction) const {
    glm::vec3 supportA = _shapeA->getSupportVertex(direction);
    glm::vec3 directionB = _transformAtoB.transformDirection(-direction);
    glm::vec3 supportB = _shapeB->getSupportVertex(directionB);
    glm::vec3 transformedB = _transformBtoA.transform(supportB);
    return _shapeA->getSupportVertex(direction) - _transformBtoA.transform(_shapeB->getSupportVertex(_transformAtoB.transformDirection(-direction)));
}

//bool evolveSimplex(std::vector<glm::vec3>& simplex, glm::vec3& direction, const Shape& shapeA, const Shape& shapeB) {
bool evolveSimplex(std::vector<glm::vec3>& simplex, const MinkowskiSurface& minkowskiSurface) {
    // The most recent addition to the simplex is always the last element and we call this point 'A'.
    glm::vec3 A = simplex.back();

    // we assume we already have at least two points in the simplex at this point
    uint32_t n = simplex.size();
    glm::vec3 direction;
    switch (n) {
        case 2: {
            // the simplex is a line [BA]
            glm::vec3 AB = simplex[0] - A;
            if (glm::dot(AB, -A) < 0.0f) {
                // A is the best simplex
                simplex[0] = A;
                // search toward the origin from A
                direction = -A;
                simplex.pop_back();
            } else {
                // the line is the best simplex
                // search perpendicular to line, toward the origin
                direction = glm::cross(glm::cross(A, AB), AB);
                if (glm::length2(direction) == 0.0f) {
                    // origin is on line, therefore it is inside simplex
                    return false;
                }
            }
        }
        break;
        case 3: {
            // the simplex is a triangle [CBA]
            // the triangleNormal is defined using the right-hand rule, winding A to B to C
            // and that will be our new direction
            direction = glm::cross(simplex[1] - A, simplex[0] - A);
            if (glm::length2(direction) == 0.0f) {
                // origin is in the plane
                return false;
            }
            if (glm::dot(direction, -A) < 0.0f) {
                // the triangleNormal points away from the origin so we reverse triangle winding
                // before searching for the next point
                direction = -direction;
                A = simplex[0];
                simplex[0] = simplex[1];
                simplex[1] = A;
            }
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
                direction = glm::cross(B - A, C - A);
                float thisDot = glm::dot(direction, -triangleCenter);
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

    // search for new point on simplex
    A = minkowskiSurface.getSupportVertex(direction);

    // validate that point
    if (glm::dot(direction, A) < 0.0f) {
        // latest point on simplex is not beyond origin along direction
        // we clear simplex to signal NO INTERSECTION
        simplex.clear();
        return false;
    }

    // continue the search
    simplex.push_back(A);
    return true;
}

#ifdef GJK_DEBUG
void printSimplex(const std::vector<glm::vec3>& points) {
    for (uint32_t i = 0; i < points.size(); ++i) {
        std::cout << i << " : " << points[i] << std::endl;
    }
}
#endif // GJK_DEBUG

bool gjk::intersect(const Transform& transformA, const Shape& shapeA, const Transform& transformB, const Shape& shapeB) {
    // The GJK algorithm works as follows:
    //
    // When two convex shapes overlap: the surface of their Minkowski sum contains the origin.
    // So the problem becomes: How to find a sub-volume of the Minkowski sum that contains the origin?
    // If such a sub-volume exists then the shapes overlap, otherwise they don't.
    //
    // The 'simplex' is a list of points on the Minkowski surface that define the sub-volume.
    // It starts with a single point.  We search toward the origin for a new point on the Minkowski surface and
    // check to see if the origin is inside the sub-volume.  We continue to grow the simplex, starting with
    // a single point, then a line-segment (2 points), then a triangle (3), and finally a tetrahedron (4).
    // Each new point is obtained by searching the Minkowski surface toward the origin, and each time we get
    // a new simplex we check to see if we can prove the origin is inside or outside.

    MinkowskiSurface minkowskiSurface(transformA, &shapeA, transformB, &shapeB);

    glm::vec3 direction = minkowskiSurface.getStartDirection();
    const float MIN_CENTROID_SEPARATION_SQUARED = 1.0e-10f;
    float distance2 = glm::length2(direction);
    if (distance2 < MIN_CENTROID_SEPARATION_SQUARED) {
        return true;
    }

    glm::vec3 B = minkowskiSurface.getSupportVertex(direction);
    if (glm::dot(direction, B) < 0.0f) {
        // when the new point on the Minkowski surface is not on the other side of the origin
        // along the direction of the search --> we immediately know the two shapes cannot possibly overlap
        return false;
    }
    direction = -direction;
    glm::vec3 A = minkowskiSurface.getSupportVertex(direction);
    if (glm::dot(direction, A) < 0.0f) {
        return false;
    }

    std::vector<glm::vec3> simplex;
    simplex.push_back(B);
    simplex.push_back(A);

    const uint32_t MAX_NUM_LOOPS = 10;
    uint32_t numLoops = 0;
    while (evolveSimplex(simplex, minkowskiSurface)) {
        if (++numLoops > MAX_NUM_LOOPS) {
            // could not prove origin is inside Minkowski surface
            return false;
        }
    }
    // an empty simplex is used to signal NO intersection
    return !simplex.empty();
}
