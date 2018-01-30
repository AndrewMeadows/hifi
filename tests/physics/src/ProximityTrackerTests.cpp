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
#include <SharedUtil.h>


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
        std::cout << "debug" << std::endl;
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
void ProximityTrackerTests::benchmark() {
    uint32_t numObjects[] = { 100, 1000, 10000, 100000 };
    uint32_t numTests = 4;
    std::vector<uint64_t> timeToAddAll;
    std::vector<uint64_t> timeToRemoveAll;
    std::vector<uint64_t> timeToMoveView;
    std::vector<uint64_t> timeToMoveObjects;
    for (uint32_t i = 0; i < numTests; ++i) {

        ProximityTracker tracker;

        // build the regions
        float pathRadius = WORLD_WIDTH / 10.0f;
        glm::vec3 regionPosition = pathRadius * glm::vec3(1.0f, 0.0f, 0.0f);
        std::vector<uint32_t> regionKeys;
        std::vector<float> regionRadiuses;
        uint32_t numRegions = 0;
        {
            std::vector<ProximityTracker::RegionType> regionTypes;
            regionTypes.push_back(ProximityTracker::REGION_1);
            regionTypes.push_back(ProximityTracker::REGION_2);
            regionTypes.push_back(ProximityTracker::REGION_3);

            numRegions = regionTypes.size();
            float radiusStep = (1.0f / (float)(numRegions + 1)) * WORLD_WIDTH;
            for (uint32_t j = 0; j < numRegions; ++j) {
                float radius = (float)(j + 1) * radiusStep;
                regionRadiuses.push_back(radius);
                uint32_t key = tracker.addRegion(regionPosition, regionRadiuses[j], regionTypes[j]);
                regionKeys.push_back(key);
            }
        }

        // build the objects
        uint32_t n = numObjects[i];
        std::vector<glm::vec3> objectPositions;
        std::vector<float> objectRadiuses;
        generatePositions(n, objectPositions);
        generateRadiuses(n, objectRadiuses);
        std::vector<uint32_t> objectKeys;
        objectKeys.reserve(n);

        // measure time to put objects in the tracker
        uint64_t startTime = usecTimestampNow();
        for (uint32_t j = 0; j < n; ++j) {
            uint32_t key = tracker.addObject(objectPositions[j], objectRadiuses[j]);
            objectKeys.push_back(key);
        }
        uint64_t usec = usecTimestampNow() - startTime;
        timeToAddAll.push_back(usec);

        // move regions around: ~1m steps in a circle
        float perStepAngle = 1.0f / pathRadius;
        uint32_t numPathSteps = 100;
        std::vector<glm::vec3> pathPositions;
        pathPositions.reserve(numPathSteps);
        glm::vec3 x(pathRadius, 0.0f, 0.0f);
        glm::vec3 z(0.0f, 0.0f, pathRadius);
        float angle = perStepAngle;
        for (uint32_t j = 0; j < numPathSteps; ++j) {
            angle += perStepAngle;
            glm::vec3 newPosition = cosf(angle) * x + sinf(angle) * z;
            pathPositions.push_back(newPosition);
        }
        std::vector<ProximityTracker::Change> changes;
        startTime = usecTimestampNow();
        for (uint32_t j = 0; j < numPathSteps; ++j) {
            for (uint32_t k = 0; k < numRegions; ++k) {
                tracker.updateRegion(regionKeys[k], pathPositions[j], regionRadiuses[k]);
            }
            tracker.collide();
            changes.clear();
            tracker.getChanges(changes);
        }
        usec = usecTimestampNow() - startTime;
        timeToMoveView.push_back(usec / numPathSteps);

        // move every 10th object around
        const float objectSpeed = 1.0f;
        std::vector<glm::vec3> newPositions;
        uint32_t numMovingObjects = n / 10;
        uint32_t jstep = n / numMovingObjects;
        uint32_t maxJ = numMovingObjects * jstep - 1;
        glm::vec3 direction;
        for (uint32_t j = 0; j < maxJ - jstep; j += jstep) {
            direction = glm::normalize(objectPositions[j + jstep] - objectPositions[j]);
            newPositions.push_back(objectPositions[j] + objectSpeed * direction);
        }
        direction = glm::normalize(objectPositions[0] - objectPositions[maxJ - jstep]);
        newPositions.push_back(objectPositions[maxJ - jstep] + objectSpeed * direction);
        uint32_t k = 0;
        changes.clear();
        startTime = usecTimestampNow();
        for (uint32_t j = 0; j < maxJ; j += jstep) {
            tracker.updateObject(objectKeys[j], newPositions[++k], objectRadiuses[j]);
        }
        tracker.collide();
        tracker.getChanges(changes);
        usec = usecTimestampNow() - startTime;
        timeToMoveObjects.push_back(usec);

        // measure time to remove objects from tracker
        startTime = usecTimestampNow();
        for (uint32_t j = 0; j < n; ++j) {
            tracker.removeObject(objectKeys[j]);
        }
        usec = usecTimestampNow() - startTime;
        timeToRemoveAll.push_back(usec);
    }

    std::cout << "[numObjects, timeToAddAll] = [" << std::endl;
    for (uint32_t i = 0; i < timeToAddAll.size(); ++i) {
        uint32_t n = numObjects[i];
        std::cout << "    " << n << ", " << timeToAddAll[i] << std::endl;
    }
    std::cout << "];" << std::endl;

    std::cout << "[numObjects, timeToMoveView] = [" << std::endl;
    for (uint32_t i = 0; i < timeToMoveView.size(); ++i) {
        uint32_t n = numObjects[i];
        std::cout << "    " << n << ", " << timeToMoveView[i] << std::endl;
    }
    std::cout << "];" << std::endl;

    std::cout << "[numObjects, timeToMoveObjects] = [" << std::endl;
    for (uint32_t i = 0; i < timeToMoveObjects.size(); ++i) {
        uint32_t n = numObjects[i];
        std::cout << "    " << n << "/10, " << timeToMoveObjects[i] << std::endl;
    }
    std::cout << "];" << std::endl;

    std::cout << "[numObjects, timeToRemoveAll] = [" << std::endl;
    for (uint32_t i = 0; i < timeToRemoveAll.size(); ++i) {
        uint32_t n = numObjects[i];
        std::cout << "    " << n << ", " << timeToRemoveAll[i] << std::endl;
    }
    std::cout << "];" << std::endl;
}

void ProximityTrackerTests::benchmarkDeadSimple() {
    uint32_t numObjects[] = { 100, 1000, 10000, 100000 };
    uint32_t numTests = 4;
    std::vector<uint64_t> timeToAddAll;
    std::vector<uint64_t> timeToRemoveAll;
    std::vector<uint64_t> timeToMoveView;
    for (uint32_t i = 0; i < numTests; ++i) {

        // build the regions
        float pathRadius = WORLD_WIDTH / 10.0f;
        glm::vec3 regionPosition = pathRadius * glm::vec3(1.0f, 0.0f, 0.0f);
        std::vector<float> regionRadiuses;
        uint32_t numRegions = 0;
        {
            std::vector<ProximityTracker::RegionType> regionTypes;
            regionTypes.push_back(ProximityTracker::REGION_1);
            regionTypes.push_back(ProximityTracker::REGION_2);
            regionTypes.push_back(ProximityTracker::REGION_3);

            numRegions = regionTypes.size();
            float radiusStep = (1.0f / (float)(numRegions + 1)) * WORLD_WIDTH;
            for (uint32_t j = 0; j < numRegions; ++j) {
                float radius = (float)(j + 1) * radiusStep;
                regionRadiuses.push_back(radius);
            }
        }

        // build the objects
        uint32_t n = numObjects[i];
        std::vector<glm::vec3> objectPositions;
        std::vector<float> objectRadiuses;
        generatePositions(n, objectPositions);
        generateRadiuses(n, objectRadiuses);

        // measure time to categorize all objects
        std::vector<uint8_t> categories;
        categories.resize(n);
        uint64_t startTime = usecTimestampNow();
        for (uint32_t j = 0; j < n; ++j) {
            float distance2 = glm::distance2(regionPosition, objectPositions[j]);
            uint8_t k;
            for (k = 0; k < 3; ++k) {
                float touchingDistance = regionRadiuses[k] + objectRadiuses[j];
                if (distance2 < touchingDistance * touchingDistance) {
                    break;
                }
            }
            categories[j] = k;
        }
        uint64_t usec = usecTimestampNow() - startTime;
        timeToAddAll.push_back(usec);

        // move regions around: ~1m steps in a circle
        float perStepAngle = 1.0f / pathRadius;
        uint32_t numPathSteps = 100;
        std::vector<glm::vec3> pathPositions;
        pathPositions.reserve(numPathSteps);
        glm::vec3 x(pathRadius, 0.0f, 0.0f);
        glm::vec3 z(0.0f, 0.0f, pathRadius);
        float angle = perStepAngle;
        for (uint32_t j = 0; j < numPathSteps; ++j) {
            angle += perStepAngle;
            glm::vec3 newPosition = cosf(angle) * x + sinf(angle) * z;
            pathPositions.push_back(newPosition);
        }
        std::vector<uint32_t> changes;
        std::vector<uint8_t> oldCategories;
        oldCategories.resize(n);
        startTime = usecTimestampNow();
        for (uint32_t j = 0; j < numPathSteps; ++j) {
            changes.clear();
            regionPosition = pathPositions[j];
            oldCategories.swap(categories);
            for (uint32_t k = 0; k < n; ++k) {
                float distance2 = glm::distance2(regionPosition, objectPositions[k]);
                uint8_t c;
                for (c = 0; c < 3; ++c) {
                    float touchingDistance = regionRadiuses[c] + objectRadiuses[k];
                    if (distance2 < touchingDistance * touchingDistance) {
                        break;
                    }
                }
                categories[k] = c;
                if (categories[k] != oldCategories[k]) {
                    changes.push_back(k);
                }
            }
        }
        usec = usecTimestampNow() - startTime;
        timeToMoveView.push_back(usec / numPathSteps);
    }

    std::cout << "simple [numObjects, timeToAddAll] = [" << std::endl;
    for (uint32_t i = 0; i < timeToAddAll.size(); ++i) {
        uint32_t n = numObjects[i];
        std::cout << "    " << n << ", " << timeToAddAll[i] << std::endl;
    }
    std::cout << "];" << std::endl;

    std::cout << "simple [numObjects, timeToMoveView] = [" << std::endl;
    for (uint32_t i = 0; i < timeToMoveView.size(); ++i) {
        uint32_t n = numObjects[i];
        std::cout << "    " << n << ", " << timeToMoveView[i] << std::endl;
    }
    std::cout << "];" << std::endl;
}
#endif // MANUAL_TEST
