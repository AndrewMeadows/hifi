//
//  DynamicCompoundShape.h
//  libraries/physics/src
//
//  Created by Andrew Meadows 2015.07.31
//  Copyright 2015 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_DynamicCompoundShape_h
#define hifi_DynamicCompoundShape_h

#include <btBulletDynamicsCommon.h>


// DynamicCompoundShape is a btCompoundShape modified to efficiently handle constant updates to child transforms.
// We exchange broadphase slop (more false positives from the broadphase) for lazier updates to the internal
// bounding box tree.
//
// Subshape transforms can be changed, which may or may not expand the total Aabb of the complete shape.  When
// such a movement expands the total Aabb then aabbIsDirty() will return true.  External code must detect this
// condition and re-add the corresponding RigidBody to the world and call clearDirtyAabb().
//
// The total Aabb can only expand not shrink, so when the internal orbits of the individual parts are bounded
// the total Aabb will eventually stop growing and the need to re-add to the world will likewise cease.

ATTRIBUTE_ALIGNED16(class) DynamicCompoundShape : public btCompoundShape {
public:
    DynamicCompoundShape(const int initialChildCapacity = 0) : btCompoundShape(true, initialChildCapacity), _aabbIsDirty(true) { }
    ~DynamicCompoundShape();

    bool aabbIsDirty() const { return _aabbIsDirty; }
    void clearDirtyAabb() { _aabbIsDirty = false; }

    // these methods wrap non-virtual methods from btCompoundShape
    void addDynamicChildShape(const btTransform& localTransform, const btCollisionShape* shape); // instead of addChildShape()
    void removeDynamicChildShapeByIndex(int childShapeIndex);
    void updateDynamicChildTransform(int childIndex, const btTransform& newChildTransform); // instead of updateChildTransform()

    // these methods override from btCompoundShape
    virtual void recalculateLocalAabb() override;
    virtual void setLocalScaling(const btVector3& scaling) override;
    virtual const char* getName() const override { return "DynamicCompound"; }

    // these for unit tests
    btScalar getChildAabbExpansionFactor() const;
    btScalar getMinChildAabbSlop() const;

private:
    bool _aabbIsDirty = true;
};

#endif // hifi_DynamicCompoundShape_h
