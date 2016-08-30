//
//  SweepProbe.h
//  libraries/physcis/src
//
//  Created by Andrew Meadows 2016.08.26
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_SweepProbe_h
#define hifi_SweepProbe_h

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>


class btCollisionWorld;

class SweepProbe : public btPairCachingGhostObject {
public:
    SweepProbe() { }
    ~SweepProbe();

    void getCollisionGroupAndMask(int16_t& group, int16_t& mask);

    void setOverlapMargin(float margin);

    void setCollisionShape(btCollisionShape* shape) override;

    void setCollisionFilterGroupAndMask(int16_t group, int16_t mask);

    void setCollisionWorld(btCollisionWorld* world);

    void measurePenetration();

    bool sweep(const btConvexShape* shape, const btTransform& start, const btTransform& end);

protected:
    void removeFromWorld();
    void addToWorld();
    void refreshOverlappingPairCache();
    //void updateShape();

protected:
    btCollisionWorld* _world { nullptr };
    btScalar _overlapMargin { 0.0f };
    int16_t _collisionFilterGroup { 0 };
    int16_t _collisionFilterMask { 0 };
    bool _inWorld { false };
};

#endif // hifi_SweepProbe_h
