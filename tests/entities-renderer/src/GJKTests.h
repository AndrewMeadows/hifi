//
//  GJKTests.h
//  tests/physics/src
//
//  Created by Virendra Singh on 2015.03.02
//  Copyright 2014 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_GJKTests_h
#define hifi_GJKTests_h

#include <QtTest/QtTest>

// Relative error macro (see errorTest in BulletTestUtils.h)
#define QCOMPARE_WITH_RELATIVE_ERROR(actual, expected, relativeError) \
    QCOMPARE_WITH_LAMBDA(actual, expected, errorTest(actual, expected, relativeError))


// Testcase class
class GJKTests : public QObject {
    Q_OBJECT
private slots:
    void intersectCubes();
    void intersectSemiRandomHulls();
};

#endif // hifi_GJKTests_h
