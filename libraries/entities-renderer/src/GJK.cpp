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

#include "GJK.h"
#include <iostream> // adebug
#include <StreamUtils.h> // adebug

#include <glm/gtx/norm.hpp>

using namespace gjk;

ConvexHull::ConvexHull(const std::vector<glm::vec3>& localVertices) {
    assert(!localVertices.empty());
    _vertices = localVertices;
    computeCentroid();
}

void ConvexHull::setVertices(const std::vector<glm::vec3>& localVertices) {
    _vertices = localVertices;
    computeCentroid();
}

void ConvexHull::setWorldTransform(const glm::vec3& translation, const glm::quat& rotation) {
    _rotation = rotation;
    _translation = translation;
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

bool evolveSimplex(std::vector<glm::vec3>& simplex, glm::vec3& direction, const ConvexHull& hullA, const ConvexHull& hullB) {
    // The GJK algorithm works as follows:
    //
    // When two convex shapes overlap their Minkowski sum contains the origin.  So the problem becomes:
    // try to find volume defined by points on the Minkowski sum surface that contains the origin.  If
    // we succeed then the shapes overlap, otherwise they don't.

    // The 'simplex' is the list of points that define the volume.
    // The most recent addition to the simplex is always the last element and we call this point 'A'.
    glm::vec3 A = simplex.back();

    // Each new point on the simplex must extend past the origin along the search direction,
    // else the two shapes cannot possibly overlap.
    if (glm::dot(direction, A) < 0.0f) {
        // latest point on simplex is not beyond origin along direction
        // we clear simplex to signal NO INTERSECTION
        simplex.clear();
        return false;
    }

    // The simplex can have from one to four points.  We add a new point by searching the Minkowski surface
    // for values that are on the other side of the origin from the current simplex.  We discard points
    // when we find better ones.  Ultimately we hope to find a tetrahedron that contains the origin,
    // or prove that such doesn't exist.

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
            simplex.push_back(hullA.getSupportVertex(direction) - hullB.getSupportVertex(-direction));
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

            // the simplest search direction is toward the origin from the center of the triangle
            direction = -(simplex[0] + simplex[1] + simplex[2]);
            A = hullA.getSupportVertex(direction) - hullB.getSupportVertex(-direction);

            // However each search may supply a point that duplicates one already in the simplex.
            // (The more continous the search on the Minkowski sum the less likely this is to happen.
            // In other words: this happens more often when the shapes involved have a small number of
            // "support points".)
            // When this happens we try alternate search directions hoping to find a "new" point.
            if (A == simplex[0] || A == simplex[1] || A == simplex[2]) {
                // search toward origin
                direction = -A;
                A = hullA.getSupportVertex(direction) - hullB.getSupportVertex(-direction);
                if (A == simplex[0] || A == simplex[1] || A == simplex[2]) {
                    // search along triangle normal
                    direction = triangleNormal;
                    A = hullA.getSupportVertex(direction) - hullB.getSupportVertex(-direction);
                    if (A == simplex[0] || A == simplex[1] || A == simplex[2]) {
                        // give up!
                        // we clear simplex to signal NO INTERSECTION
                        simplex.clear();
                        return false;
                    }
                }
            }
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

                // search for a new point but avoid adding duplicates
                direction = -triangleCenter;
                A = hullA.getSupportVertex(direction) - hullB.getSupportVertex(-direction);
                if (A == simplex[0] || A == simplex[1] || A == simplex[2]) {
                    direction = -A;
                    A = hullA.getSupportVertex(direction) - hullB.getSupportVertex(-direction);
                    if (A == simplex[0] || A == simplex[1] || A == simplex[2]) {
                        direction = triangleNormal;
                        A = hullA.getSupportVertex(direction) - hullB.getSupportVertex(-direction);
                        if (A == simplex[0] || A == simplex[1] || A == simplex[2]) {
                            // give up!
                            // clear simplex to signal NO INTERSECTION
                            simplex.clear();
                            return false;
                        }
                    }
                }
                simplex.push_back(A);
                break;
            }
            if (i == 3) {
                // origin was behind ALL new faces
                return true;
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

bool gjk::intersect(const ConvexHull& hullA, const ConvexHull& hullB) {
    glm::vec3 dir = hullA.getCentroid() - hullB.getCentroid();
    const float MIN_CENTROID_SEPARATION_SQUARED = 1.0e-10f;
    float distance2 = glm::length2(dir);
    if (distance2 < MIN_CENTROID_SEPARATION_SQUARED) {
        return true;
    }

    glm::vec3 B = hullA.getSupportVertex(dir) - hullB.getSupportVertex(-dir);
    if (glm::dot(dir, B) < 0.0f) {
        return false;
    }
    dir = -dir;
    glm::vec3 A = hullA.getSupportVertex(dir) - hullB.getSupportVertex(-dir);
    if (glm::dot(dir, A) < 0.0f) {
        return false;
    }

    std::vector<glm::vec3> simplex;
    simplex.push_back(B);
    simplex.push_back(A);

    const uint32_t MAX_NUM_LOOPS = 10;
    uint32_t loopCount = 0;
    while (evolveSimplex(simplex, dir, hullA, hullB)) {
        ++loopCount;
        if (loopCount > MAX_NUM_LOOPS) {
            // When the GJK algorithm fails to find a solution quickly the most likely case
            // is the search algorithm is having trouble identifying positive intersection,
            // so we assume this case.  This assumption only fails for near-miss intersections
            // which means we are accepting approximate shapes that is are slightly larger than
            // their true bounds.
            break;
        }
    }

    // an empty simplex is used to signal NO intersection
    return !simplex.empty();
}
