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

#ifndef hifi_ProximityTracker_h
#define hifi_ProximityTracker_h

#include <unordered_map>
#include <vector>

#include <btBulletDynamicsCommon.h>
#include <glm/glm.hpp>

// ProximityTracker uses a btDbvtBroadphase to provide fast collision checks between "things" in the world
// and "regions" of interest that represent a veiw into the world.  After each collision pass objects will
// be categorized as: "near", "mid", "far" or "very far".

class ProximityTracker {
    class Object {
        friend class ProximityTracker;
    public:
        Object() {}
        Object(const glm::vec3 position, float radius) : _position(position), _radius(radius) {}
        virtual bool noteOverlap(Object* other);
    protected:
        glm::vec3 _position { 0.0f, 0.0f, 0.0f };
        btBroadphaseProxy* _proxy { nullptr };
        float _radius { 0.0f };
        uint32_t _mapKey { 0 };
        uint8_t _type  { 0 };
        uint8_t _category { 0 };
        uint8_t _prevCategory { 0 };
    };

public:
    enum RegionType {
        REGION_1 = 1,
        REGION_2 = 2,
        REGION_3 = 4
    };

    using ObjectIndexMap = std::unordered_map<uint32_t, uint32_t>;
    using ObjectArray = std::vector<Object>;

    class OverlapCallback : public btOverlapCallback {
    public:
        OverlapCallback();
        bool processOverlap(btBroadphasePair& pair) override;
        void setObjectArray(ObjectArray* array);
        void clearRegionTotals();
    private:
        std::vector<uint32_t> _numObjectsInRegion;
        ObjectArray* _externalObjectArray { nullptr };
    };

    class Dispatcher : public btDispatcher {
    public:
        // unused overrides
        btCollisionAlgorithm* findAlgorithm(
                const btCollisionObjectWrapper* body0Wrap,
                const btCollisionObjectWrapper* body1Wrap,
                btPersistentManifold* sharedManifold=0) override { return nullptr; }
        btPersistentManifold* getNewManifold(const btCollisionObject* b0, const btCollisionObject* b1) override { return nullptr; }
        void releaseManifold(btPersistentManifold* manifold) override {}
        void clearManifold(btPersistentManifold* manifold) override {}
        bool needsCollision(const btCollisionObject* body0, const btCollisionObject* body1) override { return false; }
        bool needsResponse(const btCollisionObject* body0, const btCollisionObject* body1) override { return false; }
        void dispatchAllCollisionPairs(
                btOverlappingPairCache* pairCache,
                const btDispatcherInfo& dispatchInfo,
                btDispatcher* dispatcher) override {}
        int32_t getNumManifolds() const override { return 0; }
        btPersistentManifold* getManifoldByIndexInternal(int32_t index) override { return nullptr; }
        btPersistentManifold** getInternalManifoldPointer() override { return nullptr; }
        btPoolAllocator* getInternalManifoldPool() override { return nullptr; }
        const btPoolAllocator* getInternalManifoldPool() const override { return nullptr; }

        // used overrides
        void* allocateCollisionAlgorithm(int32_t size) override;
        void freeCollisionAlgorithm(void* ptr) override;

        // custom methods
        void dispatchAllCollisionPairs(btOverlappingPairCache* pairCache);
        void setObjectArray(ObjectArray* array);
    private:
        OverlapCallback _overlapCallback;
    };


    ProximityTracker() { _dispatcher.setObjectArray(&_objects); }
    ~ProximityTracker() {}

    uint32_t addRegion(const glm::vec3& position, float radius, RegionType type);
    void removeRegion(uint32_t key);
    void updateRegion(uint32_t key, const glm::vec3& position, float radius);

    uint32_t addObject(const glm::vec3& position, float radius);
    void removeObject(uint32_t key);
    void updateObject(uint32_t key, const glm::vec3& position, float radius);

    void collide();
    void rollCategories();
    uint32_t getNumOverlaps() const;
    uint32_t countChanges() const;
    uint32_t countTouches() const;

private:
    uint32_t createProxy(Object& object, int16_t group, int16_t mask);

    btDbvtBroadphase _broadphase;
    Dispatcher _dispatcher;
    ObjectIndexMap _objectMap;
    ObjectArray _objects;
    uint32_t _nextUniqueMapKey { 1 };
};

#endif // hifi_ProximityTracker_h
