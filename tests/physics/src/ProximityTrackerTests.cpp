//
//  ProximityTrackerTests.cpp
//  tests/physics/src
//
//  Created by Andrew Meadows on 2017.01.26
//  Copyright 2017 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "ProximityTrackerTests.h"

#include <iostream>

#include <btBulletDynamicsCommon.h>

#include <ProximityTracker.h>
#include <StreamUtils.h>


const float INV_SQRT_3 = 1.0f / sqrtf(3.0f);

QTEST_MAIN(ProximityTrackerTests)

void ProximityTrackerTests::testOverlaps() {
    ProximityTracker tracker;

    /* DEBUG helper: save for later
    std::function<void (ProximityTracker*)> printResults = [](ProximityTracker* bounds) {
        std::cout << "debug  "
            << "  numOverlaps = " << bounds->getNumOverlaps()
            << "  numTouches = " << bounds->countTouches()
            << "  numChanges = " << bounds->countChanges()
            << std::endl;
        std::cout << "debug" << std::endl;     // adebug
    };
    */

    glm::vec3 position1(0.0f, 0.0f, 0.0f);
    glm::vec3 positionA(0.0f, 1.0f, 0.0f);
    glm::vec3 positionB(0.0f, 1.0f, 0.0f);
    float radius1 = 1.0f;
    float radiusA = 1.0f;
    float radiusB = radiusA;

    // add overlapping objects
    uint32_t key1 = tracker.addRegion(position1, radius1, ProximityTracker::REGION_1);
    uint32_t keyA = tracker.addObject(positionA, radiusA);
    uint32_t keyB = tracker.addObject(positionB, radiusB);
    {
        // verify they touch
        tracker.collide();
        uint32_t expectedNumOverlaps = 2;
        uint32_t expectedNumTouches = 2;
        uint32_t expectedNumChanges = 2;
        QCOMPARE(expectedNumOverlaps, tracker.getNumOverlaps());
        QCOMPARE(expectedNumTouches, tracker.countTouches());
        QCOMPARE(expectedNumChanges, tracker.countChanges());
        tracker.rollCategories();
    }

    { // move A but keep it barely in overlap
        positionA = position1 + (0.99f * (radius1 + radiusA)) * (positionA - position1);
        tracker.updateObject(keyA, positionA, radiusA);
        // verify A still overlaps region
        tracker.collide();
        uint32_t expectedNumOverlaps = 2;
        uint32_t expectedNumTouches = 2;
        uint32_t expectedNumChanges = 0;
        QCOMPARE(expectedNumOverlaps, tracker.getNumOverlaps());
        QCOMPARE(expectedNumTouches, tracker.countTouches());
        QCOMPARE(expectedNumChanges, tracker.countChanges());
        tracker.rollCategories();
    }

    { // shrink the region a little bit
        radius1 *= 0.8f;
        tracker.updateObject(key1, position1, radius1);
        // verify region no longer overlaps A
        tracker.collide();
        uint32_t expectedNumOverlaps = 1;
        uint32_t expectedNumTouches = 1;
        uint32_t expectedNumChanges = 1;
        QCOMPARE(expectedNumOverlaps, tracker.getNumOverlaps());
        QCOMPARE(expectedNumTouches, tracker.countTouches());
        QCOMPARE(expectedNumChanges, tracker.countChanges());
        tracker.rollCategories();
    }

    { // move B out of overlap
        positionB = position1 + (1.5f * (radius1 + radiusB)) * (positionB - position1);
        tracker.updateObject(keyB, positionB, radiusB);
        // verify they no longer overlap
        tracker.collide();
        uint32_t expectedNumOverlaps = 0;
        uint32_t expectedNumTouches = 0;
        uint32_t expectedNumChanges = 1;
        QCOMPARE(expectedNumOverlaps, tracker.getNumOverlaps());
        QCOMPARE(expectedNumTouches, tracker.countTouches());
        QCOMPARE(expectedNumChanges, tracker.countChanges());
        tracker.rollCategories();
    }

    { // grow the region a lot
        float distanceA = glm::distance(position1, positionA) - radiusA;
        float distanceB = glm::distance(position1, positionB) - radiusB;
        radius1 = glm::max(distanceA, distanceB) + 0.001f;
        tracker.updateObject(key1, position1, radius1);
        // verify objects overlap again
        tracker.collide();
        uint32_t expectedNumOverlaps = 2;
        uint32_t expectedNumTouches = 2;
        uint32_t expectedNumChanges = 2;
        QCOMPARE(expectedNumOverlaps, tracker.getNumOverlaps());
        QCOMPARE(expectedNumTouches, tracker.countTouches());
        QCOMPARE(expectedNumChanges, tracker.countChanges());
        tracker.rollCategories();
    }

    { // move A to corner overlap just barely touching
        float distanceA = (radius1 + radiusA - 0.001f);
        glm::vec3 offsetA(INV_SQRT_3);
        positionA = position1 + distanceA * offsetA;
        tracker.updateObject(keyA, positionA, radiusA);
        // verify objects overlap but don't touch
        tracker.collide();
        uint32_t expectedNumOverlaps = 2;
        uint32_t expectedNumTouches = 2;
        uint32_t expectedNumChanges = 0;
        QCOMPARE(expectedNumOverlaps, tracker.getNumOverlaps());
        QCOMPARE(expectedNumTouches, tracker.countTouches());
        QCOMPARE(expectedNumChanges, tracker.countChanges());
        tracker.rollCategories();
    }

    { // move A to corner overlap just barely touching
        float distanceA = (radius1 + radiusA + 0.01f);
        glm::vec3 offsetA(INV_SQRT_3);
        positionA = position1 + distanceA * offsetA;
        tracker.updateObject(keyA, positionA, radiusA);
        // verify objects overlap but don't touch
        tracker.collide();
        uint32_t expectedNumOverlaps = 2;
        uint32_t expectedNumTouches = 1;
        uint32_t expectedNumChanges = 1;
        QCOMPARE(expectedNumOverlaps, tracker.getNumOverlaps());
        QCOMPARE(expectedNumTouches, tracker.countTouches());
        QCOMPARE(expectedNumChanges, tracker.countChanges());
        tracker.rollCategories();
    }
}

const float WORLD_WIDTH = 1000.0f;
const float MIN_RADIUS = 1.0f;
const float MAX_RADIUS = 100.0f;

float randomFloat() {
    return 2.0f * ((float)rand() / (float)RAND_MAX) - 1.0f;
}

glm::vec3 randomVec3() {
    glm::vec3 v(randomFloat(), randomFloat(), randomFloat());
    return v;
}

void generatePositions(uint32_t numObjects, std::vector<glm::vec3>& positions) {
    positions.reserve(numObjects);
    for (uint32_t i = 0; i < numObjects; ++i) {
        positions.push_back(WORLD_WIDTH * randomVec3());
    }
}

void generateRadiuses(uint32_t numRadiuses, std::vector<float>& radiuses) {
    radiuses.reserve(numRadiuses);
    for (uint32_t i = 0; i < numRadiuses; ++i) {
        radiuses.push_back(MIN_RADIUS + (MAX_RADIUS - MIN_RADIUS) * randomFloat());
    }
}

#ifdef MANUAL_TEST
void ProximityTrackerTests::benchmarkConstruction() {
    uint32_t numObjects[] = { 100, 1000, 10000, 100000, 200000, 400000 };
    uint32_t numSteps = 6;
    std::cout << "[numObjects, timeToConstruct] = [" << std::endl;
    for (uint32_t i = 0; i < numSteps; ++i) {
        uint32_t n = numObjects[i];
        std::vector<glm::vec3> positions;
        std::vector<float> radiuses;

        generatePositions(n, positions);
        generateRadiuses(n, radiuses);

        ProximityTracker tracker;
        glm::vec3 position1(0.0f, 0.0f, 0.0f);
        float radius1 = 0.25f * WORLD_WIDTH;
        float radius2 = 0.50f * WORLD_WIDTH;
        float radius3 = 0.75f * WORLD_WIDTH;
        tracker.addRegion(position1, radius1, ProximityTracker::REGION_1);
        tracker.addRegion(position1, radius2, ProximityTracker::REGION_2);
        tracker.addRegion(position1, radius3, ProximityTracker::REGION_3);

        btClock timer;
        for (uint32_t j = 0; j < n; ++j) {
            tracker.addObject(positions[j], radiuses[j]);
        }
        uint64_t msec = timer.getTimeMilliseconds();
        std::cout << "    " << n << ", " << ((float)msec / 1000.0f) << std::endl;
    }
    std::cout << "];" << std::endl;
}
#endif // MANUAL_TEST
