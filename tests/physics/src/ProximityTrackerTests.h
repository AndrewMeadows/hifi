//
//  ProximityTrackerTests.h
//  tests/physics/src
//
//  Created by Andrew Meadows on 2017.01.26
//  Copyright 2017 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_ProximityTrackerTests_h
#define hifi_ProximityTrackerTests_h

#include <QtTest/QtTest>

#define MANUAL_TEST

class ProximityTrackerTests : public QObject {
    Q_OBJECT

private slots:
    void testOverlaps();
#ifdef MANUAL_TEST
    void benchmarkConstruction();
#endif // MANUAL_TEST
};

#endif // hifi_ProximityTrackerTests_h
