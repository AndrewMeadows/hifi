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

#include <glm/gtx/norm.hpp>

const float ALMOST_ZERO = 1.0e-10f;

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
    return _translation + _rotation * _vertices[_vertices.size() - 1];
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

// experimental
glm::vec3 ConvexHull::getSmoothedSupportVertex(const glm::vec3& direction) const {
    // transform direction into local frame
    glm::vec3 dir = glm::inverse(_rotation) * direction;

    float totalWeight = 0.0f;
    glm::vec3 weightedSum(0.0f);
    float maxDot = glm::dot(dir, _vertices[0]);
    if (maxDot > 0.0f) {
        totalWeight = maxDot;
        weightedSum = (maxDot * maxDot) * _vertices[0];
    }
    float j = 0;
    uint32_t numVertices = _vertices.size() - 1; // avoid centroid at end
    for (uint32_t i = 1; i < numVertices; ++i) {
        float dot = glm::dot(dir, _vertices[i]);
        if (dot > 0.0f) {
            totalWeight += dot;
            weightedSum += (dot * dot) * _vertices[i];
        } else if (totalWeight == 0.0f && dot > maxDot) {
            j = i;
            maxDot = dot;
        }
    }
    // transform result into world-frame
    if (totalWeight > 0.0f) {
        return _translation + _rotation * ((1.0f / totalWeight) * weightedSum);
    } else {
        return _translation + _rotation * _vertices[j];
    }
}

bool ConvexHull::containsPoint(const glm::vec3& point) const {
    std::vector<glm::vec3> vertices;
    vertices.push_back(point);
    ConvexHull pointHull(vertices);
    return intersect(*this, pointHull);
}

void ConvexHull::printVerticesWorldFrame() const {
#ifdef FOO
    uint32_t numVertices = _vertices.size() - 1;
    for (uint32_t i = 0; i < numVertices; ++i) {
        std::cout << "adebug  " << i << "  " << getVertexWorldFrame(i) << std::endl;     // adebug
    }
#endif // FOO
}

void ConvexHull::computeCentroid() {
    // compute average point and store on the end of _vertices
    uint32_t numVerts = _vertices.size();
    if (numVerts > 0) {
        glm::vec3 centroid = _vertices[0];
        for (uint32_t i = 1; i < numVerts; ++i) {
            centroid += _vertices[i];
        }
        _vertices.push_back(centroid / (float)(numVerts));
    }
}


void handleTriangleSimplex(std::vector<glm::vec3> simplex, glm::vec3& dir) {
    // simplex is guaranteed to have three values [C B A] with triangle winding ABC
    // and "origin dot normal" is positive
    glm::vec3 A = simplex[2];
    glm::vec3 leg = simplex[1] - A;
    glm::vec3 triangleNormal = glm::cross(leg, simplex[0] - A);
    if (glm::dot(glm::cross(triangleNormal, leg), -A) < 0.0f) {
        if (glm::dot(leg, -A) > 0.0f) {
            // line AB is the new simplex
            simplex[0] = simplex[1];
            simplex[1] = A;
            simplex.pop_back();
            dir = glm::cross(glm::cross(A, leg), leg);
        } else {
            // A is the new simplex
            simplex[0] = A;
            simplex.pop_back();
            simplex.pop_back();
            dir = -A;
        }
    } else {
        leg = simplex[0] - A;
        if (glm::dot(glm::cross(leg, triangleNormal), -A) > 0.0f) {
            if (glm::dot(leg, -A) > 0.0f) {
                // line AC is the new simplex
                simplex[1] = A;
                simplex.pop_back();
                dir = glm::cross(glm::cross(A, leg), leg);
            } else {
                // A is the new simplex
                simplex[0] = A;
                simplex.pop_back();
                simplex.pop_back();
                dir = -A;
            }
        } else {
            dir = triangleNormal;
        }
    }
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

    bool running = true;
    int loopCount = 0;
    while (running) {
        A = simplex.back();
        if (glm::dot(dir, A) < 0.0f) {
            // latest point on simplex is not beyond origin along dir
            // which means the origin is outside the Minkowski sum
            // which means the two hulls don't overlap
            simplex.clear();
            running = false;
            break;
        }
        uint32_t n = simplex.size();
        switch (n) {
            case 2: {
                // line
                glm::vec3 AB = simplex[0] - A;
                if (glm::dot(AB, -A) < 0.0f) {
                    simplex[0] = A;
                    dir = -A;
                    simplex.pop_back();
                } else {
                    dir = glm::cross(glm::cross(A, AB), AB);
                }
                if (glm::length2(dir) == 0.0f) {
                    // origin is on line
                    running = false;
                    break;
                }
                simplex.push_back(hullA.getSupportVertex(dir) - hullB.getSupportVertex(-dir));
            }
            break;
            case 3: {
                // triangle
                glm::vec3 triangleNormal = glm::cross(simplex[1] - A, simplex[0] - A);
                if (glm::length2(triangleNormal) == 0.0f) {
                    // origin is in the plane
                    running = false;
                    break;
                }
                if (glm::dot(triangleNormal, -A) < 0.0f) {
                    // reverse triangle winding so normal (ABC) points toward origin
                    triangleNormal = -triangleNormal;
                    dir = simplex[0];
                    simplex[0] = simplex[1];
                    simplex[1] = dir;
                }
                //handleTriangleSimplex(simplex, dir);
                dir = -(simplex[0] + simplex[1] + simplex[2]);
                simplex.push_back(hullA.getSupportVertex(dir) - hullB.getSupportVertex(-dir));
            }
            break;
            case 4: {
                // tetrahedron
                uint32_t i = 0;
                for (i = 0; i < 3; ++i) {
                    uint32_t j = (i + 1) % 3;
                    glm::vec3 B = simplex[j];
                    glm::vec3 C = simplex[i];
                    glm::vec3 triangleCenter = A + B + C;
                    float thisDot = glm::dot(glm::cross(B - A, C - A), -triangleCenter);
                    if (thisDot < 0.0f) {
                        continue;
                    }
                    simplex[0] = C;
                    simplex[1] = B;
                    simplex[2] = A;
                    simplex.pop_back();
                    dir = -(simplex[0] + simplex[1] + simplex[2]);
                    simplex.push_back(hullA.getSupportVertex(dir) - hullB.getSupportVertex(-dir));
                }
                if (i == 3) {
                    running = false;
                }
                break;
            }
            break;
            default: {
                simplex.clear();
                running = false;
            }
            break;
        }
        ++loopCount;
        if (loopCount > 10) {
            simplex.clear();
            break;
        }
    }
    return !simplex.empty();
}
