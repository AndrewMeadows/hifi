//
//  GJKTests.cpp
//  tests/physics/src
//
//  Created by Virendra Singh on 2015.03.02
//  Copyright 2014 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "GJKTests.h"

#include <iostream> // adebug
#include <GJK.h>

// Add additional qtest functionality (the include order is important!)
#include "GJKTests.h"
#include <test-utils/QTestExtensions.h>
#include <StreamUtils.h> // adebug


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

void GJKTests::intersectCubes() {
    const float CUBE_SIDE = 1.0f;
    std::vector<glm::vec3> scaledCube;
    scaledCube.reserve(CUBE_VERTICES.size());
    for (uint32_t i = 0; i < CUBE_VERTICES.size(); ++i) {
        scaledCube.push_back((0.5f * CUBE_SIDE) * CUBE_VERTICES[i]);
    }
    gjk::ConvexHull hullA(scaledCube);
    gjk::ConvexHull hullB(scaledCube);
    const float DELTA = 1.0e-6f;

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
        }
    }

    {   // cubes should NOT intersect just outside faces and corners
        float offsetScale = CUBE_SIDE * (1.0f + DELTA);
        for (uint32_t i = 0; i < CUBE_VERTICES_AND_FACES.size(); ++i) {
            linearOffset = offsetScale * CUBE_VERTICES_AND_FACES[i];
            hullB.setWorldTransform(linearOffset, angularOffset);
            QVERIFY(!gjk::intersect(hullA, hullB));
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
        // DOWN
        linearOffset = offsetScale * glm::vec3(0.0f, 0.0f, -1.0f);
        hullB.setWorldTransform(linearOffset, angularOffset);
        hullB.printVerticesWorldFrame();
        QVERIFY(gjk::intersect(hullA, hullB));

        // shift hullB so its two vertical tips almost intersect hullA
        offsetScale = 0.5f * (1.0f + SQRT_THREE) * CUBE_SIDE * (1.0f + DELTA);
        // UP
        linearOffset = offsetScale * glm::vec3(0.0f, 0.0f, 1.0f);
        hullB.setWorldTransform(linearOffset, angularOffset);
        QVERIFY(!gjk::intersect(hullA, hullB));
        // DOWN
        linearOffset = offsetScale * glm::vec3(0.0f, 0.0f, -1.0f);
        hullB.setWorldTransform(linearOffset, angularOffset);
        QVERIFY(!gjk::intersect(hullA, hullB));
    }
}

void GJKTests::intersectSemiRandomHulls() {
//#define FOO
#ifdef FOO
    std::cout << "adebug  start intersectSemiRandomHulls()" << std::endl;     // adebug
    const float DELTA = 1.0e-1f;

    /*
    // make two semi-random octagonal hulls (one point per cubic face)
    std::vector<glm::vec3> scaledHull;
    scaledHull.reserve(CUBE_FACES.size());
    scaledHull.push_back(2.3f * CUBE_FACES[0]);
    scaledHull.push_back(3.5f * CUBE_FACES[1]);
    scaledHull.push_back(4.1f * CUBE_FACES[2]);
    scaledHull.push_back(4.7f * CUBE_FACES[3]);
    scaledHull.push_back(3.9f * CUBE_FACES[4]);
    scaledHull.push_back(2.6f * CUBE_FACES[5]);
    gjk::ConvexHull hullA(scaledHull);

    scaledHull[0] = 3.3f * CUBE_FACES[0];
    scaledHull[1] = 4.5f * CUBE_FACES[1];
    scaledHull[2] = 3.1f * CUBE_FACES[2];
    scaledHull[3] = 4.7f * CUBE_FACES[3];
    scaledHull[4] = 3.9f * CUBE_FACES[4];
    scaledHull[5] = 4.6f * CUBE_FACES[5];
    gjk::ConvexHull hullB(scaledHull);
    */
    // make two semi-random octagonal hulls (one point per cubic face)
    std::vector<glm::vec3> scaledHull;
    scaledHull.reserve(CUBE_FACES.size());
    scaledHull.push_back(1.1f * CUBE_FACES[0]);
    scaledHull.push_back(1.2f * CUBE_FACES[1]);
    scaledHull.push_back(1.3f * CUBE_FACES[2]);
    scaledHull.push_back(1.4f * CUBE_FACES[3]);
    scaledHull.push_back(1.5f * CUBE_FACES[4]);
    scaledHull.push_back(1.6f * CUBE_FACES[5]);
    gjk::ConvexHull hullA(scaledHull);
    gjk::ConvexHull hullB(scaledHull);

    // give hullB an arbitrary fixed transform
    /*
    glm::vec3 linearOffset = glm::vec3(-2.3f, 4.7f, -3.9f);
    float angle = 0.725f;
    glm::vec3 axis = glm::normalize(glm::vec3(1.2f, 2.3f, 4.5f));
    glm::quat angularOffset = glm::angleAxis(angle, axis);
    hullB.setWorldTransform(linearOffset, angularOffset);
    */
    glm::vec3 linearOffset;
    glm::quat angularOffset;
    glm::vec3 axis;

    uint32_t POINTS_PER_TRIANGLE = 3;
    uint32_t numFaces = OCTAGON_TRIANGLE_INDICES.size() / POINTS_PER_TRIANGLE;
    // for each face of hullB:
    for (uint32_t i = 0; i < numFaces; ++i) {
        // select a semi-random point on the face
        uint32_t triangleIndex = i * POINTS_PER_TRIANGLE;
        glm::vec3 point0 = hullB.getVertexWorldFrame(OCTAGON_TRIANGLE_INDICES[triangleIndex]);
        glm::vec3 point1 = hullB.getVertexWorldFrame(OCTAGON_TRIANGLE_INDICES[triangleIndex + 1]);
        glm::vec3 point2 = hullB.getVertexWorldFrame(OCTAGON_TRIANGLE_INDICES[triangleIndex + 2]);
        //glm::vec3 facePoint = 0.1f * point0 + 0.2f * point1 + 0.7f * point2;
        glm::vec3 facePoint = (point0 + point1 + point2) / 3.0f;
        // compute the face normal
        glm::vec3 faceNormal = glm::normalize(glm::cross(point1 - point0, point2 - point0));

        // for each vertex of hullA
        uint32_t numVertices = scaledHull.size();
        for (uint32_t j = 0; j < numVertices; ++j) {
            glm::vec3 vertexJ = hullA.getVertex(j);
            float distanceJ = glm::length(vertexJ);
            // compute the linear offset to hullA
            linearOffset = facePoint + (1.0f - DELTA) * distanceJ * faceNormal;

            // compute the angularOffset to bring vertexJ around to facePoint
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

            // transform hullA such that its vertex is just barely NOT intersecting hullB's face
            linearOffset = facePoint + (1.0f + DELTA) * distanceJ * faceNormal;
            hullA.setWorldTransform(linearOffset, angularOffset);
            QVERIFY(!gjk::intersect(hullA, hullB));
        }
    }
#endif // FOO
}


#if 0
void MeshMassPropertiesTests::testTetrahedron(){
    // given the four vertices of a tetrahedron verify the analytic formula for inertia
    // agrees with expected results

    // these numbers from the Tonon paper:
    btVector3 points[4];
    points[0] = btVector3(8.33220f, -11.86875f, 0.93355f);
    points[1] = btVector3(0.75523f, 5.00000f, 16.37072f);
    points[2] = btVector3(52.61236f, 5.00000f, -5.38580f);
    points[3] = btVector3(2.00000f, 5.00000f, 3.00000f);

    btScalar expectedVolume = 1873.233236f;

    btMatrix3x3 expectedInertia;
    expectedInertia[0][0] = 43520.33257f;
    expectedInertia[1][1] = 194711.28938f;
    expectedInertia[2][2] = 191168.76173f;
    expectedInertia[1][2] = -4417.66150f;
    expectedInertia[2][1] = -4417.66150f;
    expectedInertia[0][2] = 46343.16662f;
    expectedInertia[2][0] = 46343.16662f;
    expectedInertia[0][1] = -11996.20119f;
    expectedInertia[1][0] = -11996.20119f;

    // compute volume
    btScalar volume = computeTetrahedronVolume(points);
    btScalar error = (volume - expectedVolume) / expectedVolume;
    if (fabsf(error) > acceptableRelativeError) {
        std::cout << __FILE__ << ":" << __LINE__ << " ERROR : volume of tetrahedron off by = "
            << error << std::endl;
    }

    btVector3 centerOfMass = 0.25f * (points[0] + points[1] + points[2] + points[3]);

    // compute inertia tensor
    // (shift the points so that tetrahedron's local centerOfMass is at origin)
    for (int i = 0; i < 4; ++i) {
        points[i] -= centerOfMass;
    }
    btMatrix3x3 inertia;
    computeTetrahedronInertia(volume, points, inertia);

    QCOMPARE_WITH_ABS_ERROR(volume, expectedVolume, acceptableRelativeError * volume);

    QCOMPARE_WITH_RELATIVE_ERROR(inertia, expectedInertia, acceptableRelativeError);
}

void MeshMassPropertiesTests::testOpenTetrahedonMesh() {
    // given the simplest possible mesh (open, with one triangle)
    // verify MeshMassProperties computes the right nubers

    // these numbers from the Tonon paper:
    VectorOfPoints points;
    points.push_back(btVector3(8.33220f, -11.86875f, 0.93355f));
    points.push_back(btVector3(0.75523f, 5.00000f, 16.37072f));
    points.push_back(btVector3(52.61236f, 5.00000f, -5.38580f));
    points.push_back(btVector3(2.00000f, 5.00000f, 3.00000f));

    btScalar expectedVolume = 1873.233236f;

    btMatrix3x3 expectedInertia;
    expectedInertia[0][0] = 43520.33257f;
    expectedInertia[1][1] = 194711.28938f;
    expectedInertia[2][2] = 191168.76173f;
    expectedInertia[1][2] = -4417.66150f;
    expectedInertia[2][1] = -4417.66150f;
    expectedInertia[0][2] = 46343.16662f;
    expectedInertia[2][0] = 46343.16662f;
    expectedInertia[0][1] = -11996.20119f;
    expectedInertia[1][0] = -11996.20119f;

    // test as an open mesh with one triangle
    VectorOfPoints shiftedPoints;
    shiftedPoints.push_back(points[0] - points[0]);
    shiftedPoints.push_back(points[1] - points[0]);
    shiftedPoints.push_back(points[2] - points[0]);
    shiftedPoints.push_back(points[3] - points[0]);
    VectorOfIndices triangles;
    pushTriangle(triangles, 1, 2, 3);
    btVector3 expectedCenterOfMass = 0.25f * (shiftedPoints[0] + shiftedPoints[1] + shiftedPoints[2] + shiftedPoints[3]);

    // compute mass properties
    MeshMassProperties mesh(shiftedPoints, triangles);

    // verify
    // (expected - actual) / expected > e   ==>  expected - actual  >  e * expected
    QCOMPARE_WITH_ABS_ERROR(mesh._volume, expectedVolume, acceptableRelativeError * expectedVolume);
    QCOMPARE_WITH_ABS_ERROR(mesh._centerOfMass, expectedCenterOfMass, acceptableAbsoluteError);
    QCOMPARE_WITH_RELATIVE_ERROR(mesh._inertia, expectedInertia, acceptableRelativeError);
}

void MeshMassPropertiesTests::testClosedTetrahedronMesh() {
    // given a tetrahedron as a closed mesh of four tiangles
    // verify MeshMassProperties computes the right nubers

    // these numbers from the Tonon paper:
    VectorOfPoints points;
    points.push_back(btVector3(8.33220f, -11.86875f, 0.93355f));
    points.push_back(btVector3(0.75523f, 5.00000f, 16.37072f));
    points.push_back(btVector3(52.61236f, 5.00000f, -5.38580f));
    points.push_back(btVector3(2.00000f, 5.00000f, 3.00000f));

    btScalar expectedVolume = 1873.233236f;

    btMatrix3x3 expectedInertia;
    expectedInertia[0][0] = 43520.33257f;
    expectedInertia[1][1] = 194711.28938f;
    expectedInertia[2][2] = 191168.76173f;
    expectedInertia[1][2] = -4417.66150f;
    expectedInertia[2][1] = -4417.66150f;
    expectedInertia[0][2] = 46343.16662f;
    expectedInertia[2][0] = 46343.16662f;
    expectedInertia[0][1] = -11996.20119f;
    expectedInertia[1][0] = -11996.20119f;

    btVector3 expectedCenterOfMass = 0.25f * (points[0] + points[1] + points[2] + points[3]);

    VectorOfIndices triangles;
    pushTriangle(triangles, 0, 2, 1);
    pushTriangle(triangles, 0, 3, 2);
    pushTriangle(triangles, 0, 1, 3);
    pushTriangle(triangles, 1, 2, 3);

    // compute mass properties
    MeshMassProperties mesh(points, triangles);

    // verify
    QCOMPARE_WITH_ABS_ERROR(mesh._volume, expectedVolume, acceptableRelativeError * expectedVolume);
    QCOMPARE_WITH_ABS_ERROR(mesh._centerOfMass, expectedCenterOfMass, acceptableAbsoluteError);
    QCOMPARE_WITH_RELATIVE_ERROR(mesh._inertia, expectedInertia, acceptableRelativeError);

    // test again, but this time shift the points so that the origin is definitely OUTSIDE the mesh
    btVector3 shift = points[0] + expectedCenterOfMass;
    for (int i = 0; i < (int)points.size(); ++i) {
        points[i] += shift;
    }
    expectedCenterOfMass = 0.25f * (points[0] + points[1] + points[2] + points[3]);

    // compute mass properties
    mesh.computeMassProperties(points, triangles);

    // verify
//    QCOMPARE_WITH_ABS_ERROR(mesh._volume, expectedVolume, acceptableRelativeError * expectedVolume);
//    QCOMPARE_WITH_ABS_ERROR(mesh._centerOfMass, expectedCenterOfMass, acceptableAbsoluteError);
//    QCOMPARE_WITH_RELATIVE_ERROR(mesh._inertia, expectedInertia, acceptableRelativeError);
}

void MeshMassPropertiesTests::testBoxAsMesh() {
    // verify that a mesh box produces the same mass properties as the analytic box.

    // build a box:
    //                            /
    //                           y
    //                          /
    //            6-------------------------7
    //           /|                        /|
    //          / |                       / |
    //         /  2----------------------/--3
    //        /  /                      /  /
    //   |   4-------------------------5  /  --x--
    //   z   | /                       | /
    //   |   |/                        |/
    //       0 ------------------------1

    btScalar x(5.0f);
    btScalar y(3.0f);
    btScalar z(2.0f);

    VectorOfPoints points;
    points.reserve(8);

    points.push_back(btVector3(0.0f, 0.0f, 0.0f));
    points.push_back(btVector3(x, 0.0f, 0.0f));
    points.push_back(btVector3(0.0f, y, 0.0f));
    points.push_back(btVector3(x, y, 0.0f));
    points.push_back(btVector3(0.0f, 0.0f, z));
    points.push_back(btVector3(x, 0.0f, z));
    points.push_back(btVector3(0.0f, y, z));
    points.push_back(btVector3(x, y, z));

    VectorOfIndices triangles;
    pushTriangle(triangles, 0, 1, 4);
    pushTriangle(triangles, 1, 5, 4);
    pushTriangle(triangles, 1, 3, 5);
    pushTriangle(triangles, 3, 7, 5);
    pushTriangle(triangles, 2, 0, 6);
    pushTriangle(triangles, 0, 4, 6);
    pushTriangle(triangles, 3, 2, 7);
    pushTriangle(triangles, 2, 6, 7);
    pushTriangle(triangles, 4, 5, 6);
    pushTriangle(triangles, 5, 7, 6);
    pushTriangle(triangles, 0, 2, 1);
    pushTriangle(triangles, 2, 3, 1);

    // compute expected mass properties analytically
    btVector3 expectedCenterOfMass = 0.5f * btVector3(x, y, z);
    btScalar expectedVolume = x * y * z;
    btMatrix3x3 expectedInertia;
    computeBoxInertia(expectedVolume, btVector3(x, y, z), expectedInertia);

    // compute the mass properties using the mesh
    MeshMassProperties mesh(points, triangles);

    // verify

    QCOMPARE_WITH_ABS_ERROR(mesh._volume, expectedVolume, acceptableRelativeError * expectedVolume);
    QCOMPARE_WITH_ABS_ERROR(mesh._centerOfMass, expectedCenterOfMass, acceptableAbsoluteError);

    // test this twice: _RELATIVE_ERROR doesn't test zero cases (to avoid divide-by-zero); _ABS_ERROR does.
    QCOMPARE_WITH_ABS_ERROR(mesh._inertia, expectedInertia, acceptableAbsoluteError);
    QCOMPARE_WITH_RELATIVE_ERROR(mesh._inertia, expectedInertia, acceptableRelativeError);

    // These two macros impl this:
//    for (int i = 0; i < 3; ++i) {
//        for (int j = 0; j < 3; ++j) {
//            if (expectedInertia [i][j] == btScalar(0.0f)) {
//                error = mesh._inertia[i][j] - expectedInertia[i][j];                                  // COMPARE_WITH_ABS_ERROR
//                if (fabsf(error) > acceptableAbsoluteError) {
//                    std::cout << __FILE__ << ":" << __LINE__ << " ERROR : inertia[" << i << "][" << j << "] off by "
//                        << error << " absolute"<< std::endl;
//                }
//            } else {
//                error = (mesh._inertia[i][j] - expectedInertia[i][j]) / expectedInertia[i][j];        // COMPARE_WITH_RELATIVE_ERROR
//                if (fabsf(error) > acceptableRelativeError) {
//                    std::cout << __FILE__ << ":" << __LINE__ << " ERROR : inertia[" << i << "][" << j << "] off by "
//                        << error << std::endl;
//                }
//            }
//        }
//    }
}
#endif // 0
