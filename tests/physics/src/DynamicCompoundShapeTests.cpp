//
//  DynamicCompoundShapeTests.cpp
//  tests/physics/src
//
//  Created by Andrew Meadows on 2016.03.14
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "DynamicCompoundShapeTests.h"

#include <iostream> // adebug
#include <DynamicCompoundShape.h>
#include <ShapeInfo.h>
#include <ShapeManager.h>

#include "BulletTestUtils.h"
#include "../QTestExtensions.h"

QTEST_MAIN(DynamicCompoundShapeTests)

//const btTransform identityTransform;

void DynamicCompoundShapeTests::test001() {
    ShapeManager shapeManager;

    float acceptableError = 1.0e-6f;
    btTransform identityTransform;
    identityTransform.setRotation(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f));
    identityTransform.setOrigin(btVector3(0.0f, 0.0f, 0.0f));

    // create three spheres with different radiuses
    float radius0 = 1.0f;
    float radius1 = 2.0f;
    float radius2 = 3.0f;
    ShapeInfo info;
    info.setSphere(radius0);
    const btCollisionShape* shape0 = shapeManager.getShape(info);
    info.setSphere(radius1);
    const btCollisionShape* shape1 = shapeManager.getShape(info);
    info.setSphere(radius2);
    const btCollisionShape* shape2 = shapeManager.getShape(info);

    btVector3 position0(-(radius0 + radius1), 0.0f, 0.0f);
    btVector3 position1(0.0f, 0.0f, 0.0f);
    btVector3 position2(radius1 + radius2, 0.0f, 0.0f);

    btQuaternion rotation;
    btTransform transform;
    transform.setRotation(rotation);

    DynamicCompoundShape dynamicShape;

    // add three spheres to one DynamicCompoundShape and verify number of children
    transform.setOrigin(position0);
    dynamicShape.addDynamicChildShape(transform, shape0);
    QCOMPARE(dynamicShape.getNumChildShapes(), 1);
    QVERIFY(dynamicShape.aabbIsDirty());

    transform.setOrigin(position1);
    dynamicShape.addDynamicChildShape(transform, shape1);
    QCOMPARE(dynamicShape.getNumChildShapes(), 2);
    QVERIFY(dynamicShape.aabbIsDirty());

    transform.setOrigin(position2);
    dynamicShape.addDynamicChildShape(transform, shape2);
    QCOMPARE(dynamicShape.getNumChildShapes(), 3);
    QVERIFY(dynamicShape.aabbIsDirty());

    // verify that we can clean the dirty bit
    dynamicShape.clearDirtyAabb();
    QVERIFY(!dynamicShape.aabbIsDirty());

    // verify the Aabb of the compound shape has the expected dimensions
    btVector3 oldAabbMin, oldAabbMax;
    dynamicShape.getAabb(identityTransform, oldAabbMin, oldAabbMax);
    btVector3 oldDiagonal = oldAabbMax - oldAabbMin;

    // it's hard to compute the exact expected puffed up Aabb for the compound shape,
    // but we can compute reasonable upper and lower bounds

    btVector3 lowerBound(radius0 + radius1 + radius2, radius2, radius2);
    lowerBound = 2.0f * lowerBound;
    QVERIFY(oldDiagonal[0] > lowerBound[0]);
    QVERIFY(oldDiagonal[1] > lowerBound[1]);
    QVERIFY(oldDiagonal[2] > lowerBound[2]);

    btVector3 upperBound = lowerBound;
    upperBound *= (1.0f + dynamicShape.getChildAabbExpansionFactor());
    QVERIFY(oldDiagonal[0] < upperBound[0]);
    QVERIFY(oldDiagonal[1] < upperBound[1]);
    QVERIFY(oldDiagonal[2] < upperBound[2]);

    { // verify that a small motion of one child does not set the dirty bit
        const btScalar shift = 0.5f * dynamicShape.getChildAabbExpansionFactor();
        btVector3 newPosition0 = position0 + btVector3(-shift, 0.0f, 0.0f);
        transform.setOrigin(newPosition0);
        dynamicShape.updateDynamicChildTransform(0, transform); // zeroth child
        QVERIFY(!dynamicShape.aabbIsDirty());

        // and that its Aabb has not changed
        btVector3 newAabbMin, newAabbMax;
        dynamicShape.getAabb(identityTransform, newAabbMin, newAabbMax);
        QCOMPARE_WITH_ABS_ERROR(newAabbMin, oldAabbMin, acceptableError);
        QCOMPARE_WITH_ABS_ERROR(newAabbMax, oldAabbMax, acceptableError);
    }

    { // verify that a large motion outward of one child does set the dirty bit
        const btScalar shift = 1.5f * dynamicShape.getChildAabbExpansionFactor();
        btVector3 newPosition0 = position0 + btVector3(-shift, 0.0f, 0.0f);
        transform.setOrigin(newPosition0);
        dynamicShape.updateDynamicChildTransform(0, transform); // zeroth child
        QVERIFY(dynamicShape.aabbIsDirty());

        // and that its Aabb has expanded
        btVector3 newAabbMin, newAabbMax;
        dynamicShape.getAabb(identityTransform, newAabbMin, newAabbMax);
        btVector3 newDiagonal = newAabbMax - newAabbMin;
        QVERIFY(newDiagonal[0] > oldDiagonal[0]);
        QCOMPARE_WITH_ABS_ERROR(newDiagonal[1], oldDiagonal[1], acceptableError);
        QCOMPARE_WITH_ABS_ERROR(newDiagonal[2], oldDiagonal[2], acceptableError);

        dynamicShape.clearDirtyAabb();
    }

    // recompute Aabb
    dynamicShape.getAabb(identityTransform, oldAabbMin, oldAabbMax);
    oldDiagonal = oldAabbMax - oldAabbMin;

    { // verify that child's return to old position does not set the dirty bit
        transform.setOrigin(position0);
        dynamicShape.updateDynamicChildTransform(0, transform); // zeroth child
        QVERIFY(!dynamicShape.aabbIsDirty());

        // and that its Aabb has not changed
        btVector3 newAabbMin, newAabbMax;
        dynamicShape.getAabb(identityTransform, newAabbMin, newAabbMax);
        QCOMPARE_WITH_ABS_ERROR(newAabbMin, oldAabbMin, acceptableError);
        QCOMPARE_WITH_ABS_ERROR(newAabbMax, oldAabbMax, acceptableError);
    }

    int maxNumChildren = dynamicShape.getNumChildShapes();

    { // remove the first shape
        dynamicShape.removeDynamicChildShapeByIndex(0);

        // verify that its number of children has decreased
        QCOMPARE(dynamicShape.getNumChildShapes(), maxNumChildren - 1);

        // and that none of them are shape0
        for (int i = 0; i < dynamicShape.getNumChildShapes(); ++i) {
            QVERIFY(dynamicShape.getChildShape(i) != shape0);
        }

        // verify its Aabb is NOT dirty
        QVERIFY(!dynamicShape.aabbIsDirty());

        // and that it has not changed
        btVector3 newAabbMin, newAabbMax;
        dynamicShape.getAabb(identityTransform, newAabbMin, newAabbMax);
        QCOMPARE_WITH_ABS_ERROR(newAabbMin, oldAabbMin, acceptableError);
        QCOMPARE_WITH_ABS_ERROR(newAabbMax, oldAabbMax, acceptableError);
    }

    { // verify we can recalculate Aabb
        dynamicShape.recalculateLocalAabb();

        // verify Aabb is marked as dirty
        QVERIFY(dynamicShape.aabbIsDirty());

        // verify that the Aabb is about what we expect
        btVector3 newAabbMin, newAabbMax;
        dynamicShape.getAabb(identityTransform, newAabbMin, newAabbMax);
        btVector3 newDiagonal = newAabbMax - newAabbMin;

        btVector3 lowerBound(radius1 + radius2, radius2, radius2);
        lowerBound = 2.0f * lowerBound;
        QVERIFY(newDiagonal[0] > lowerBound[0]);
        QVERIFY(newDiagonal[1] > lowerBound[1]);
        QVERIFY(newDiagonal[2] > lowerBound[2]);

        btVector3 upperBound = lowerBound;
        upperBound *= (1.0f + dynamicShape.getChildAabbExpansionFactor());
        QVERIFY(newDiagonal[0] < upperBound[0]);
        QVERIFY(newDiagonal[1] < upperBound[1]);
        QVERIFY(newDiagonal[2] < upperBound[2]);
    }
}
