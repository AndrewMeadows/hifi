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

#include <iostream> // adebug
#include "DynamicCompoundShape.h"

//#include <btBulletDynamicsCommon.h>


DynamicCompoundShape::~DynamicCompoundShape() {
    // the child shapes must be deleted by outside scope
}

const btScalar CHILD_AABB_EXPANSION_FACTOR(0.2f);
const btScalar MIN_CHILD_AABB_SLOP(0.1f);

// helper function
void computeDynamicChildAabb(
        const btTransform& localTransform,
        const btCollisionShape* shape,
        btVector3& minCorner, btVector3& maxCorner) {
    // get the Aabb of the child shape, expand it to be a cube (so it is invarient under rotations),
    // and then expand it again by some margin (so the dynamicTree doesn't need to be rebuilt as often).
    shape->getAabb(localTransform, minCorner, maxCorner);
    btVector3 center = btScalar(0.5f) * (minCorner + maxCorner);
    minCorner -= center;
    maxCorner -= center;
    btScalar maxRadius = btMax(minCorner.length(), maxCorner.length());
    maxRadius += btMax(maxRadius * CHILD_AABB_EXPANSION_FACTOR, MIN_CHILD_AABB_SLOP);

    // now that we've got the max possible radius for all rotations plus slop... compute new Aabb corners
    btVector3 diagonal(maxRadius, maxRadius, maxRadius);
    minCorner = center - diagonal;
    maxCorner = center + diagonal;
}

void DynamicCompoundShape::addDynamicChildShape(const btTransform& localTransform, const btCollisionShape* shape) {
    // we first add the shape using the base method
    addChildShape(localTransform, const_cast<btCollisionShape*>(shape));

    // compute an expanded Aabb for the child shape
    btVector3 minCorner, maxCorner;
    computeDynamicChildAabb(localTransform, shape, minCorner, maxCorner);

    // expand the total aabbMin/aabbMax
    for (int i = 0; i < 3; i++) {
        if (m_localAabbMin[i] > minCorner[i]) {
            m_localAabbMin[i] = minCorner[i];
            _aabbIsDirty = true;
        }
        if (m_localAabbMax[i] < maxCorner[i]) {
            m_localAabbMax[i] = maxCorner[i];
            _aabbIsDirty = true;
        }
    }

    int childShapeIndex = getNumChildShapes() - 1;
    btDbvtVolume bounds = btDbvtVolume::FromMM(minCorner, maxCorner);
    m_dynamicAabbTree->update(m_children[childShapeIndex].m_node, bounds);
}

void DynamicCompoundShape::removeDynamicChildShapeByIndex(int childShapeIndex) {
    assert(childShapeIndex > -1 && childShapeIndex < m_children.size());
    btCompoundShape::removeChildShapeByIndex(childShapeIndex);
}

void DynamicCompoundShape::updateDynamicChildTransform(int childShapeIndex, const btTransform& newChildTransform) {
    btAssert(childShapeIndex >= 0 && childShapeIndex < getNumChildShapes());
    btCompoundShapeChild& child = m_children[childShapeIndex];
    child.m_transform = newChildTransform;

    // assume Aabb center = shape center
    btVector3 center = m_children[childShapeIndex].m_node->volume.Center();
    btVector3 delta = newChildTransform.getOrigin() - center;
    btScalar deltaLength = delta.length();
    if (deltaLength < MIN_CHILD_AABB_SLOP) {
        // the child shape is still inside its box
        // no need to update the AabbTree
        return;
    }

    // compute a new cache center assuming that the child shape will continue
    // to move in the direction of delta
    center += ((deltaLength + btScalar(0.5f) * MIN_CHILD_AABB_SLOP) / deltaLength) * delta;

    // update the dynamic aabb tree
    btVector3 extents = m_children[childShapeIndex].m_node->volume.Extents();
    btVector3 minCorner = center - extents;
    btVector3 maxCorner = center + extents;
    ATTRIBUTE_ALIGNED16(btDbvtVolume) bounds = btDbvtVolume::FromMM(minCorner, maxCorner);
    m_dynamicAabbTree->update(m_children[childShapeIndex].m_node, bounds);

    // expand the localAabb if necessary
    for (int i = 0; i < 3; ++i) {
        if (m_localAabbMin[i] > minCorner[i]) {
            m_localAabbMin[i] = minCorner[i];
            _aabbIsDirty = true;
        }
        if (m_localAabbMax[i] < maxCorner[i]) {
            m_localAabbMax[i] = maxCorner[i];
            _aabbIsDirty = true;
        }
    }
}

// virtual
void DynamicCompoundShape::recalculateLocalAabb() {
    m_localAabbMin = btVector3(btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT));
    m_localAabbMax = btVector3(btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT));

    for(int i = 0; i < m_children.size(); ++i) {
        // update the dynamic aabb tree
        btVector3 minCorner, maxCorner;
        computeDynamicChildAabb(getChildTransform(i), m_children[i].m_childShape, minCorner, maxCorner);
        ATTRIBUTE_ALIGNED16(btDbvtVolume) bounds = btDbvtVolume::FromMM(minCorner, maxCorner);
        m_dynamicAabbTree->update(m_children[i].m_node, bounds);

        //extend the total aabbMin/aabbMax
        for (int j = 0; j < 3; j++) {
            if (m_localAabbMin[j] > minCorner[j]) {
                m_localAabbMin[j] = minCorner[j];
            }
            if (m_localAabbMax[j] < maxCorner[j]) {
                m_localAabbMax[j] = maxCorner[j];
            }
        }
    }
    _aabbIsDirty = true;
}

// virtual
void DynamicCompoundShape::setLocalScaling(const btVector3& scaling) {
    // Danger: despite the fact that this method is implemented, after looking at the Bullet code I don't have
    // faith that "local scaling" actually works for the btCompounsShape base class, so probably best if we
    // never try to use this feature until we're sure it does work.
    m_localAabbMin = btVector3(btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT));
    m_localAabbMax = btVector3(btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT));
    // TODO: assert that scaling has no zero components
    btVector3 scaleRatio = scaling / m_localScaling;

    for(int i = 0; i < m_children.size(); ++i) {
        // update the child transform
        btVector3 childScale = m_children[i].m_childShape->getLocalScaling();
        childScale = childScale * scaleRatio;
        m_children[i].m_childShape->setLocalScaling(childScale);
        btVector3 center = getChildTransform(i).getOrigin() * scaleRatio;
        m_children[i].m_transform.setOrigin(center);

        // update the dynamic aabb tree
        btVector3 extents = m_children[i].m_node->volume.Extents() * scaleRatio;
        btVector3 minCorner = center - extents;
        btVector3 maxCorner = center + extents;
        ATTRIBUTE_ALIGNED16(btDbvtVolume) bounds = btDbvtVolume::FromMM(minCorner, maxCorner);
        m_dynamicAabbTree->update(m_children[i].m_node, bounds);

        //extend the total aabbMin/aabbMax
        for (int j = 0; j < 3; j++) {
            if (m_localAabbMin[j] > minCorner[j]) {
                m_localAabbMin[j] = minCorner[j];
                _aabbIsDirty = true;
            }
            if (m_localAabbMax[j] < maxCorner[j]) {
                m_localAabbMax[j] = maxCorner[j];
                _aabbIsDirty = true;
            }
        }
    }

    m_localScaling = scaling;
}

// for unit tests
btScalar DynamicCompoundShape::getChildAabbExpansionFactor() const {
    return CHILD_AABB_EXPANSION_FACTOR;
}

// for unit tests
btScalar DynamicCompoundShape::getMinChildAabbSlop() const {
    return MIN_CHILD_AABB_SLOP;
}

