//
//  SettleAction.h
//  libraries/physcis/src
//
//  Created by Andrew Meadows 2016-10-06
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//
//  http://bulletphysics.org/Bullet/BulletFull/classbtActionInterface.html

#ifndef hifi_SettleAction_h
#define hifi_SettleAction_h

#include <stdint.h>

#include <btBulletDynamicsCommon.h>

// The SettleAction is used to slowly move each object on a list toward its final resting transform,
// the goal being to help piles of dynamic objects settle down on the remote side of a distributed
// physics simulation.

class SettleAction : public btActionInterface {
public:
    SettleAction(uint8_t maxNumBodies);
    ~SettleAction();

    void addBody(btRigidBody* body, const btTransform& transform, uint64_t expiry);
    void removeBody(btRigidBody* body);

    // these methods overridden for btActionInterface
    void updateAction( btCollisionWorld* collisionWorld, btScalar deltaTimeStep) override;
    void debugDraw(btIDebugDraw* debugDrawer) override { }

private:
    // pointers to c-style fixed-size arrays
    btRigidBody** _body { nullptr };
    btTransform* _transform { nullptr };
    uint64_t* _expiry { nullptr };
    uint8_t _numBodies { 0 };
    uint8_t _maxNumBodies { 0 };
};

#endif // hifi_SettleAction_h
