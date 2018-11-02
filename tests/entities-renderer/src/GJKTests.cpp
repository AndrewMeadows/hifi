//
//  GJKTests.cpp
//  tests/entities-renderer/src
//
//  Created by Andrew Meadows 2018.08.06
//  Copyright 2018 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "GJKTests.h"
#include <test-utils/QTestExtensions.h>

#include <GJK.h>


QTEST_MAIN(GJKTests)

std::vector<glm::vec3> CUBE_VERTICES {
    { -1.0f, -1.0f, -1.0f },
    { -1.0f, -1.0f,  1.0f },
    { -1.0f,  1.0f, -1.0f },
    { -1.0f,  1.0f,  1.0f },
    {  1.0f, -1.0f, -1.0f },
    {  1.0f, -1.0f,  1.0f },
    {  1.0f,  1.0f, -1.0f },
    {  1.0f,  1.0f,  1.0f }
};

std::vector<glm::vec3> CUBE_FACES {
    {  1.0f,  0.0f,  0.0f },
    { -1.0f,  0.0f,  0.0f },
    {  0.0f,  1.0f,  0.0f },
    {  0.0f, -1.0f,  0.0f },
    {  0.0f,  0.0f,  1.0f },
    {  0.0f,  0.0f, -1.0f }
};

std::vector<uint32_t> OCTAGON_TRIANGLE_INDICES = {
    0, 2, 4,
    0, 4, 3,
    0, 3, 5,
    0, 5, 2,
    1, 2, 5,
    1, 5, 3,
    1, 3, 4,
    1, 4, 2
};

std::vector<glm::vec3> CUBE_VERTICES_AND_FACES = {
    { -1.0f, -1.0f, -1.0f },
    { -1.0f, -1.0f,  1.0f },
    { -1.0f,  1.0f, -1.0f },
    { -1.0f,  1.0f,  1.0f },
    {  1.0f, -1.0f, -1.0f },
    {  1.0f, -1.0f,  1.0f },
    {  1.0f,  1.0f, -1.0f },
    {  1.0f,  1.0f,  1.0f },
    {  1.0f,  0.0f,  0.0f },
    { -1.0f,  0.0f,  0.0f },
    {  0.0f,  1.0f,  0.0f },
    {  0.0f, -1.0f,  0.0f },
    {  0.0f,  0.0f,  1.0f },
    {  0.0f,  0.0f, -1.0f }
};

const float DELTA = 1.0e-6f;

void GJKTests::intersectCubes() const {
    // create two cubes and move them into intersection (or not) and verify the GJK algorithm
    // correctly distinquishes the two cases.

    const float CUBE_SIDE = 1.0f;
    std::vector<glm::vec3> scaledCube;
    scaledCube.reserve(CUBE_VERTICES.size());
    for (uint32_t i = 0; i < CUBE_VERTICES.size(); ++i) {
        scaledCube.push_back((0.5f * CUBE_SIDE) * CUBE_VERTICES[i]);
    }
    gjk::ConvexHull hullA(scaledCube);
    gjk::ConvexHull hullB(scaledCube);

    glm::vec3 linearOffset(0.0f);
    glm::quat angularOffset(1.0f, 0.0f, 0.0f, 0.0f);

    {   // cube should intersect itself
        QVERIFY(gjk::intersect(hullA, hullB));
    }

    {   // cubes should intersect just inside faces and corners
        float offsetScale = CUBE_SIDE * (1.0f - DELTA);
        for (uint32_t i = 0; i < CUBE_VERTICES_AND_FACES.size(); ++i) {
            linearOffset = offsetScale * CUBE_VERTICES_AND_FACES[i];
            hullB.setWorldTransform(linearOffset, angularOffset);
            QVERIFY(gjk::intersect(hullA, hullB));
            QVERIFY(gjk::intersect(hullB, hullA));
        }
    }

    {   // cubes should NOT intersect just outside faces and corners
        float offsetScale = CUBE_SIDE * (1.0f + DELTA);
        for (uint32_t i = 0; i < CUBE_VERTICES_AND_FACES.size(); ++i) {
            linearOffset = offsetScale * CUBE_VERTICES_AND_FACES[i];
            hullB.setWorldTransform(linearOffset, angularOffset);
            QVERIFY(!gjk::intersect(hullA, hullB));
            QVERIFY(!gjk::intersect(hullB, hullA));
        }
    }

    {   // rotate hullB so it has two opposing vertices aligned along Z-axis
        const float SQRT_THREE = 1.7320508076f;
        const float ONE_OVER_SQRT_THREE = 1.0f / SQRT_THREE;
        const float ONE_OVER_SQRT_TWO = 0.7071067812f;

        float angle = acosf(ONE_OVER_SQRT_THREE);
        glm::vec3 axis = glm::vec3(ONE_OVER_SQRT_TWO, ONE_OVER_SQRT_TWO, 0.0f);
        angularOffset = glm::angleAxis(angle, axis);

        // shift hullB so its two vertical tips just barely intersects hullA
        float offsetScale = 0.5f * (1.0f + SQRT_THREE) * CUBE_SIDE * (1.0f - DELTA);
        // UP
        linearOffset = offsetScale * glm::vec3(0.0f, 0.0f, 1.0f);
        hullB.setWorldTransform(linearOffset, angularOffset);
        QVERIFY(gjk::intersect(hullA, hullB));
        QVERIFY(gjk::intersect(hullB, hullA));
        // DOWN
        linearOffset = offsetScale * glm::vec3(0.0f, 0.0f, -1.0f);
        hullB.setWorldTransform(linearOffset, angularOffset);
        QVERIFY(gjk::intersect(hullA, hullB));
        QVERIFY(gjk::intersect(hullB, hullA));

        // shift hullB so its two vertical tips almost intersect hullA
        offsetScale = 0.5f * (1.0f + SQRT_THREE) * CUBE_SIDE * (1.0f + DELTA);
        // UP
        linearOffset = offsetScale * glm::vec3(0.0f, 0.0f, 1.0f);
        hullB.setWorldTransform(linearOffset, angularOffset);
        QVERIFY(!gjk::intersect(hullA, hullB));
        QVERIFY(!gjk::intersect(hullB, hullA));
        // DOWN
        linearOffset = offsetScale * glm::vec3(0.0f, 0.0f, -1.0f);
        hullB.setWorldTransform(linearOffset, angularOffset);
        QVERIFY(!gjk::intersect(hullA, hullB));
        QVERIFY(!gjk::intersect(hullB, hullA));
    }
}

void GJKTests::intersectIrregularOctagons() const {
    // make two slightly irregular random octagonal hulls (one point per cubic face)
    std::vector<glm::vec3> points;
    points.reserve(CUBE_FACES.size());
    points.push_back(12.3f * CUBE_FACES[0]);
    points.push_back(13.5f * CUBE_FACES[1]);
    points.push_back(14.1f * CUBE_FACES[2]);
    points.push_back(14.7f * CUBE_FACES[3]);
    points.push_back(13.9f * CUBE_FACES[4]);
    points.push_back(12.6f * CUBE_FACES[5]);
    gjk::ConvexHull hullA(points);

    points[0] = 13.3f * CUBE_FACES[0];
    points[1] = 14.5f * CUBE_FACES[1];
    points[2] = 13.1f * CUBE_FACES[2];
    points[3] = 14.7f * CUBE_FACES[3];
    points[4] = 13.9f * CUBE_FACES[4];
    points[5] = 14.6f * CUBE_FACES[5];
    gjk::ConvexHull hullB(points);

    // give hullB an arbitrary fixed transform
    glm::vec3 linearOffset = glm::vec3(-2.3f, 4.7f, -3.9f);
    float angle = 0.725f;
    glm::vec3 axis = glm::normalize(glm::vec3(1.2f, 2.3f, 4.5f));
    glm::quat angularOffset = glm::angleAxis(angle, axis);
    hullB.setWorldTransform(linearOffset, angularOffset);
    glm::vec3 centroidB = hullB.getCentroid();

    const uint32_t POINTS_PER_TRIANGLE = 3;
    uint32_t numFaces = OCTAGON_TRIANGLE_INDICES.size() / POINTS_PER_TRIANGLE;
    // for each face of hullB:
    for (uint32_t i = 0; i < numFaces; ++i) {
        // compute face center
        uint32_t triangleIndex = i * POINTS_PER_TRIANGLE;
        glm::vec3 point0 = hullB.getVertex(OCTAGON_TRIANGLE_INDICES[triangleIndex]);
        glm::vec3 point1 = hullB.getVertex(OCTAGON_TRIANGLE_INDICES[triangleIndex + 1]);
        glm::vec3 point2 = hullB.getVertex(OCTAGON_TRIANGLE_INDICES[triangleIndex + 2]);
        glm::vec3 faceCenter = (point0 + point1 + point2) / 3.0f;

        // compute face normal
        glm::vec3 faceNormal = glm::normalize(glm::cross(point1 - point0, point2 - point0));

        // make sure we computed faceNormal correctly: it should point away from hull's centroid
        QVERIFY(glm::dot(faceNormal, faceCenter - centroidB) > 0.0f);

        // for each vertex of hullA:
        uint32_t numVertices = points.size();
        for (uint32_t j = 0; j < numVertices; ++j) {
            glm::vec3 vertexJ = hullA.getVertexLocalFrame(j);
            float distanceJ = glm::length(vertexJ);
            // compute linear offset to hullA such that its vertex will touch (or not) faceCenter
            linearOffset = faceCenter + (1.0f - DELTA) * distanceJ * faceNormal;

            // compute angularOffset to bring vertexJ around to faceCenter
            angularOffset = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
            float vertexDotNormal = glm::dot(vertexJ, faceNormal) / distanceJ;
            if (fabsf(vertexDotNormal) > 1.0e-9f) {
                float angle = acosf(glm::clamp(-vertexDotNormal, -1.0f, 1.0f));
                axis = glm::normalize(glm::cross(vertexJ, -faceNormal));
                angularOffset = glm::angleAxis(angle, axis);
            }

            // transform hullA such that its vertex is just barely intersecting hullB's face
            hullA.setWorldTransform(linearOffset, angularOffset);
            QVERIFY(gjk::intersect(hullA, hullB));
            QVERIFY(gjk::intersect(hullB, hullA));

            // transform hullA such that its vertex is just barely NOT intersecting hullB's face
            linearOffset = faceCenter + (1.0f + DELTA) * distanceJ * faceNormal;
            glm::vec3 tipA = linearOffset - distanceJ * faceNormal;
            QVERIFY(glm::dot(tipA - faceCenter, faceNormal) > 0.0f);
            hullA.setWorldTransform(linearOffset, angularOffset);
            QVERIFY(!gjk::intersect(hullB, hullA));
        }
    }
}

void GJKTests::containsPoint() const {
    // make slightly irregular random octagonal hull (one point per cubic face)
    std::vector<glm::vec3> points;
    points.reserve(CUBE_FACES.size());
    points.push_back(12.3f * CUBE_FACES[0]);
    points.push_back(13.5f * CUBE_FACES[1]);
    points.push_back(14.1f * CUBE_FACES[2]);
    points.push_back(14.7f * CUBE_FACES[3]);
    points.push_back(13.9f * CUBE_FACES[4]);
    points.push_back(12.6f * CUBE_FACES[5]);
    gjk::ConvexHull hull(points);

    // give it a non-identity transform
    const glm::vec3 translation = { 1.3f, -2.4f, 5.8f };
    const float angle = 0.987f;
    const float SQRT_ONE_THIRD = sqrtf(1.0f / 3.0f );
    glm::vec3 axis = { SQRT_ONE_THIRD, -SQRT_ONE_THIRD, SQRT_ONE_THIRD };
    glm::quat rotation = glm::angleAxis(angle, axis);
    hull.setWorldTransform(translation, rotation);

    glm::vec3 hullCentroid = hull.getCentroid();

    const uint32_t POINTS_PER_TRIANGLE = 3;
    uint32_t numFaces = OCTAGON_TRIANGLE_INDICES.size() / POINTS_PER_TRIANGLE;
    // for each hull face:
    for (uint32_t i = 0; i < numFaces; ++i) {
        // compute face center
        uint32_t triangleIndex = i * POINTS_PER_TRIANGLE;
        glm::vec3 point0 = hull.getVertex(OCTAGON_TRIANGLE_INDICES[triangleIndex]);
        glm::vec3 point1 = hull.getVertex(OCTAGON_TRIANGLE_INDICES[triangleIndex + 1]);
        glm::vec3 point2 = hull.getVertex(OCTAGON_TRIANGLE_INDICES[triangleIndex + 2]);
        glm::vec3 faceCenter = (point0 + point1 + point2) / 3.0f;

        // compute face normal
        glm::vec3 faceNormal = glm::normalize(glm::cross(point1 - point0, point2 - point0));

        // make sure we computed faceNormal correctly: it should point away from hull's centroid
        QVERIFY(glm::dot(faceNormal, faceCenter - hullCentroid) > 0.0f);

        // compute point just INSIDE hull on its face, and make sure hull contains it
        glm::vec3 point = faceCenter - DELTA * faceNormal;
        QVERIFY(hull.containsPoint(point));

        // compute point just OUTSIDE hull on its face, and make sure hull DOES NOT contain it
        point = faceCenter + DELTA * faceNormal;
        QVERIFY(!hull.containsPoint(point));
    }
}

