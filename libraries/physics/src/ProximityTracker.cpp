//
//  ProximityTracker.h
//  interface/src/
//
//  Created by Andrew Meadows 2018.01.29
//  Copyright 2018 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "ProximityTracker.h"

#include <functional>
#include <math.h>

#include <glm/gtx/norm.hpp>
#include <LinearMath/btPoolAllocator.h>

#include "BulletUtil.h"

const uint8_t PROXIMITY_NEAR = 0;
const uint8_t PROXIMITY_MID = 1;
const uint8_t PROXIMITY_DISTANT = 2;
const uint8_t PROXIMITY_TOO_FAR = 3;

const int16_t COLLISION_GROUP_REGION_1 = 1 << 0;
const int16_t COLLISION_GROUP_REGION_2 = 1 << 1;
const int16_t COLLISION_GROUP_REGION_3 = 1 << 2;
const int16_t COLLISION_GROUP_OBJECT = 1 << 14;
const int16_t COLLISION_MASK_REGION_1 = COLLISION_GROUP_OBJECT;
const int16_t COLLISION_MASK_REGION_2 = COLLISION_GROUP_OBJECT;
const int16_t COLLISION_MASK_REGION_3 = COLLISION_GROUP_OBJECT;
const int16_t COLLISION_MASK_OBJECT = COLLISION_GROUP_REGION_1 | COLLISION_GROUP_REGION_2 | COLLISION_GROUP_REGION_3;

const int32_t NUM_REGIONS = 4;
const int32_t REGION_INDICES[] = { -1, 0, 1, -1, 2 };
const uint32_t NUM_REGION_INDICES = 5;


ProximityTracker::OverlapCallback::OverlapCallback() {
    for (int32_t i = 0; i < NUM_REGIONS; ++i) {
        _numObjectsInRegion.push_back(0);
    }
}

bool ProximityTracker::OverlapCallback::processOverlap(btBroadphasePair& pair) {
    uint32_t index0 = (uint32_t)(uint64_t)(pair.m_pProxy0->m_clientObject);
    uint32_t index1 = (uint32_t)(uint64_t)(pair.m_pProxy1->m_clientObject);

    // the proxy stores the index into an array of Objects
    // we extract the indices to access the Objects
    assert(index0 < _externalObjectArray->size());
    assert(index1 < _externalObjectArray->size());
    Object* object0 = _externalObjectArray->data() + index0;
    Object* object1 = _externalObjectArray->data() + index1;

    // narrowphase check
    float touchDistance = object0->_radius + object1->_radius;
    if (glm::distance2(object0->_position, object1->_position) < touchDistance * touchDistance) {
        // collision group config guarantees all overlaps are between a "region" and a "thing"
        // we only need to call noteOverlap() on the "thing" (it's NO-OP when called on region)
        // and it will return "true" when we do, hence this "if" block below:
        if (!object1->noteOverlap(object0)) {
            object0->noteOverlap(object1);
        }
        // NOTE: only one of the types should be non-zero
        uint32_t type = (uint32_t)(object0->_type + object1->_type);
        assert(type < NUM_REGION_INDICES);

        // lookup the index to region tally
        int32_t index = REGION_INDICES[type];
        assert(index > -1); // should never get a negative index

        ++_numObjectsInRegion[index];
    }
    // a return value of 'true' would indicate that the btBroadphasePair should be deleted from the cache
    return false;
}

void ProximityTracker::OverlapCallback::setObjectArray(ProximityTracker::ObjectArray* array) {
    assert(array);
    _externalObjectArray = array;
}

void ProximityTracker::OverlapCallback::clearRegionTotals() {
    for (int32_t i = 0; i < NUM_REGIONS; ++i) {
        _numObjectsInRegion[i] = 0;
    }
}

bool ProximityTracker::Object::noteOverlap(Object* other) {
    if (other->_type) {
        // Object::_type is non-zero only for regions, which means this is a "thing"
        // and we want to accumulate the region's bit in its _category
        _category |=  other->_type;
        return true;
    }
    return false;
}

void* ProximityTracker::Dispatcher::allocateCollisionAlgorithm(int32_t size) {
    // The normal way Bullet uses a Dispatcher::allocateCollisionAlgorithm() is to allocate memory for one here
    // which is then filled by a CreateFunc which has already been found in a lookup matrix before this context.
    // Since we're only colliding Sphere-vs-Sphere we don't need to bother.  This is a NO-OP.
    return nullptr;
}

void ProximityTracker::Dispatcher::freeCollisionAlgorithm(void* ptr) {
    // This is the method that would normally free memory that had been allocated in a memory pool by
    // allocateCollisionAlgorithm(). For us this is a NO-OP.
}

void ProximityTracker::Dispatcher::dispatchAllCollisionPairs(btOverlappingPairCache* pairCache) {
    _overlapCallback.clearRegionTotals();
    pairCache->processAllOverlappingPairs(&_overlapCallback, this);
}

void ProximityTracker::Dispatcher::setObjectArray(ProximityTracker::ObjectArray* array) {
    assert(array);
    _overlapCallback.setObjectArray(array);
}

uint32_t ProximityTracker::addObject(const Object& object, int16_t group, int16_t mask) {
    // add a "proxy" for this object to the broadphase
    btVector3 diagonal = object._radius * btVector3(1.0f, 1.0f, 1.0f);
    btVector3 aabbMin = glmToBullet(object._position) - diagonal;
    btVector3 aabbMax = aabbMin + 2.0f * diagonal;
    int32_t shapeType = 0; // unsused by btBroadphase::createProxy()
    void* clientObject = nullptr;
    btDispatcher* dispatcher = nullptr; // unused by btBroadphase::createProxy()
    void* multiSapProxy = nullptr; // unused by btBroadphase::createProxy()
    btBroadphaseProxy* proxy = _broadphase.createProxy(aabbMin, aabbMax, shapeType,
            clientObject, group, mask, dispatcher, multiSapProxy);

    // copy the Object, set its various members, and add it to the array
    Object newObject = object;
    newObject._proxy = proxy; // object remembers pointer to its proxy
    uint32_t mapKey = _nextUniqueMapKey++;
    newObject._mapKey = mapKey;
    if (group != COLLISION_GROUP_OBJECT) {
        // the object just be a "region" which means
        // the type has same numerical value as group
        newObject._type = (uint8_t)group;
    }
    uint32_t arrayIndex = _objects.size();
    _objects.push_back(newObject);

    // proxy stores the object's arrayIndex (in a void*) which will be used in OverlapCallback::processOverlap
    // to access the Object when doing the narrow-phase collision check.  This means: when the Object moves
    // within the array we MUST update the corresponding proxy->m_clientObject.
    proxy->m_clientObject = (void*)((uint64_t)arrayIndex);

    // store the arrayIndex by unique mapKey so we can look it up quickly in a map
    // Again, when the Object moves within the array we MUST update the map entry.
    _objectMap[mapKey] = arrayIndex;

    // return mapKey to the outside as unique identifier this object
    return mapKey;
}

void ProximityTracker::removeObject(uint32_t key) {
    ObjectIndexMap::iterator itr = _objectMap.find(key);
    if (itr != _objectMap.end()) {
        // get the index
        uint32_t index = itr->second;
        assert(index < _objects.size());

        // erase from map
        _objectMap.erase(itr);

        // delete proxy
        btBroadphaseProxy* proxy = _objects[index]._proxy;
        _broadphase.getOverlappingPairCache()->cleanProxyFromPairs(proxy, &_dispatcher);
        _broadphase.destroyProxy(proxy, &_dispatcher);

        // reduce array
        assert(_objects.size() > 0);
        uint32_t lastIndex = _objects.size() - 1;
        if (index < lastIndex) {
            // copy last element to vacant spot
            _objects[index] = _objects[lastIndex];

            // update proxy's back-index
            assert(_objects[index]._proxy);
            _objects[index]._proxy->m_clientObject = (void*)((uint64_t)index);

            // update map
            itr = _objectMap.find(_objects[index]._mapKey);
            assert(itr != _objectMap.end());
            itr->second = index;
        }
        _objects.pop_back();
    }
}

void ProximityTracker::updateObject(uint32_t key, const glm::vec3& position, float radius) {
    ObjectIndexMap::iterator itr = _objectMap.find(key);
    if (itr != _objectMap.end()) {
        // get the index
        uint32_t index = itr->second;
        assert(index < _objects.size());

        Object& object = _objects[index];
        object._position = position;
        object._radius = radius;

        btVector3 diagonal = radius * btVector3(1.0f, 1.0f, 1.0f);
        btVector3 aabbMin = glmToBullet(position) - diagonal;
        btVector3 aabbMax = aabbMin + 2.0f * diagonal;
        btDispatcher* dispatcher = nullptr; // unused
        _broadphase.setAabb(object._proxy, aabbMin, aabbMax, dispatcher);
    }
}

void ProximityTracker::collide() {
    // Broadphase::collide() API takes a pointer to a Dispatcher because it may need to call
    // Dispatcher::freeCollisionAlgorithm(btBroadphasePair::m_algorithm) for any pairs that happen
    // to be culled by fancy callback logic.  The callstack for this event is something like:
    //
    // PairCache::removeOverlappingPair()
    // --> PairCache::processAllOverlappingPairs()
    //     --> foreach overlap: OverlapCallback::processOverlap()
    //     --> PairCache::removeOverlappingPair() if the callback returned 'true'
    //     --> PairCache::cleanOverlappingPair()
    //     --> Dispatcher::freeCollisionAlgorithm(btBroadphasePair::m_algorithm);)
    //
    // Where are the CollisionAlgorithms normally allocated?  For the btCollisionDispatcher the
    // callstack looks something like:
    //
    // btCollisionDispatcher::dispatchAllCollisionPairs()
    // --> Dispatcher::getNearCallback(pair, m_dispatcher, m_dispatchInfo)
    // --> foreach overlap: btCollisionPairCallback::processOverlap()
    //     --> invokes a CreateFunc that have been registered in the CollisionConfiguration
    //
    // This roundabout way of doing things with callbacks and registered CreateFuncs was done to make
    // the system modular while also making it immune to rewrites of internal containers (e.g. replace
    // an array with linkedlist or something).
    //
    // We are only colliding Sphere-vs-Sphere we don't need the CreateFunc lookup matrix for pool-
    // allocated CollisionAlgorithms, but we still need to provide a pointer to the dispatcher.
    _broadphase.collide(&_dispatcher);

    _dispatcher.dispatchAllCollisionPairs(_broadphase.getOverlappingPairCache());
}

void ProximityTracker::rollCategories() {
    // TODO? this could be achieved with a double-buffer swap and clear
    // if the categories were stored in their own arrays
    for (uint32_t i = 0; i < _objects.size(); ++i) {
        _objects[i]._prevCategory = _objects[i]._category;
        _objects[i]._category = 0;
    }
}

uint32_t ProximityTracker::getNumOverlaps() const {
    return (uint32_t)(_broadphase.getOverlappingPairCache()->getNumOverlappingPairs());
}

uint32_t ProximityTracker::countChanges() const {
    uint32_t numChanges = 0;
    for (uint32_t i = 0; i < _objects.size(); ++i) {
        if (_objects[i]._category != _objects[i]._prevCategory) {
            ++numChanges;
        }
    }
    return numChanges;
}

uint32_t ProximityTracker::countTouches() const {
    int32_t numTouches = 0;
    for (uint32_t i = 0; i < _objects.size(); ++i) {
        if (_objects[i]._category) {
            ++numTouches;
        }
    }
    return numTouches;
}

#include <iostream> // adebug
#include <StreamUtils.h> // adebug
void ProximityTracker::test() {
    std::cout << "adebug  test()" << std::endl;     // adebug

    std::function<void (ProximityTracker*)> printResults = [](ProximityTracker* bounds) {
        std::cout << "adebug  "
            << "  numOverlaps = " << bounds->getNumOverlaps()
            << "  numTouches = " << bounds->countTouches()
            << "  numChanges = " << bounds->countChanges()
            << std::endl; // adebug
        std::cout << "adebug" << std::endl;     // adebug
    };

    glm::vec3 position1(0.0f, 0.0f, 0.0f);
    float radius1 = 1.0f;
    glm::vec3 positionA(0.0f, 1.0f, 0.0f);
    glm::vec3 positionB(0.0f, 1.0f, 0.0f);
    float radiusA = 1.0f;
    float radiusB = radiusA;
    Object region1(position1, radius1);
    //glm::vec3 positionA = position1 + (radius1 + radiusA - 0.1f) * glm::vec3(1.0f);
    Object objectA(positionA, radiusA);
    Object objectB(positionB, radiusB);

    // add overlapping objects
    std::cout << "adebug  add region and objects in overlap" << std::endl;     // adebug
    uint32_t key1 = addObject(region1, COLLISION_GROUP_REGION_1, COLLISION_MASK_REGION_1);
    uint32_t keyA = addObject(objectA, COLLISION_GROUP_OBJECT, COLLISION_MASK_OBJECT);
    uint32_t keyB = addObject(objectB, COLLISION_GROUP_OBJECT, COLLISION_MASK_OBJECT);
    //std::cout << "adebug  keys = " << key1 << ", " << keyA << std::endl;     // adebug
    std::cout << "adebug  keys = " << key1 << ", " << keyA << ", " << keyB << std::endl;     // adebug
    // verify they touch
    collide();
    printResults(this);
    rollCategories();

    { // move A but keep it barely in overlap
        std::cout << "adebug  move object inside overlap" << std::endl;     // adebug
        positionA = position1 + (0.99f * (radius1 + radiusA)) * (positionA - position1);
        updateObject(keyA, positionA, radiusA);
        // verify A still overlaps region
        collide();
        printResults(this);
        rollCategories();
    }

    { // shrink the region a little bit
        std::cout << "adebug  shrink the region" << std::endl;     // adebug
        radius1 *= 0.8f;
        updateObject(key1, position1, radius1);
        // verify region no longer overlaps A
        collide();
        printResults(this);
        rollCategories();
    }

    { // move B out of overlap
        std::cout << "adebug  move object out of overlap" << std::endl;     // adebug
        positionB = position1 + (1.5f * (radius1 + radiusB)) * (positionB - position1);
        updateObject(keyB, positionB, radiusB);
        // verify they no longer overlap
        collide();
        printResults(this);
        rollCategories();
    }

    { // grow the region a lot
        std::cout << "adebug  grow the region" << std::endl;     // adebug
        float distanceA = glm::distance(position1, positionA) - radiusA;
        float distanceB = glm::distance(position1, positionB) - radiusB;
        radius1 = glm::max(distanceA, distanceB) + 0.001f;
        updateObject(key1, position1, radius1);
        // verify objects overlap again
        collide();
        printResults(this);
        rollCategories();
    }

    { // move A to corner overlap just barely touching
        std::cout << "adebug  move A into corner overlap just barely touching" << std::endl;
        float invSqrt3 = 1.0f / sqrtf(3.0f);
        float distanceA = (radius1 + radiusA - 0.001f);
        glm::vec3 offsetA(invSqrt3);
        positionA = position1 + distanceA * offsetA;
        updateObject(keyA, positionA, radiusA);
        // verify objects overlap but don't touch
        collide();
        printResults(this);
        rollCategories();
    }

    { // move A to corner overlap just barely touching
        std::cout << "adebug  move A into corner overlap but not touching" << std::endl;
        float invSqrt3 = 1.0f / sqrtf(3.0f);
        float distanceA = (radius1 + radiusA + 0.01f);
        glm::vec3 offsetA(invSqrt3);
        positionA = position1 + distanceA * offsetA;
        updateObject(keyA, positionA, radiusA);
        // verify objects overlap but don't touch
        collide();
        printResults(this);
        rollCategories();
    }
}
