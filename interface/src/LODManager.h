//
//  LODManager.h
//  interface/src/LODManager.h
//
//  Created by Clement on 1/16/15.
//  Copyright 2015 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_LODManager_h
#define hifi_LODManager_h

#include <DependencyManager.h>
#include <NumericalConstants.h>
#include <OctreeConstants.h>
#include <PIDController.h>
#include <SimpleMovingAverage.h>
#include <render/Args.h>

const float DEFAULT_TARGET_FPS = 60.0f;
const float DEFAULT_DESKTOP_TARGET_FPS = DEFAULT_TARGET_FPS;
const float DEFAULT_HMD_TARGET_FPS = 100.0f;

class AABox;

class LODManager : public QObject, public Dependency {
    Q_OBJECT
    SINGLETON_DEPENDENCY

public:
    static bool shouldRender(const RenderArgs* args, const AABox& bounds);

    Q_INVOKABLE void setAutomaticLODAdjust(bool value);
    Q_INVOKABLE bool getAutomaticLODAdjust() const { return _automaticLODAdjust; }

    Q_INVOKABLE void setMinAngularDiameter(float valueDegrees); // takes degrees
    Q_INVOKABLE float getMinAngularDiameter() const; // returns degrees

    Q_INVOKABLE float getAvgRenderTime() const { return _avgRenderTime; }
    Q_INVOKABLE QString getRenderMode() const { return _renderMode; }

    Q_INVOKABLE QString getLODFeedbackText();

    void setLODScaleFactor(float scaleFactor);
    float getLODScaleFactor() const;

    void autoAdjustLOD(float batchTimeMsec, float engineRunTimeMsec, float deltaTimeSec);

    void loadSettings();
    void saveSettings();
    void resetLODAdjust();
    void setRenderMode(QString mode);

signals:
    void LODIncreased();
    void LODDecreased();

private:
    LODManager();

    QString _renderMode { "unknown" };
    uint64_t _lodAdjustExpiry { 0 }; // next timestamp to check for LOD adjustment
    float _minAngularDiameter { 0.0f }; // radians
    float _targetFPS { DEFAULT_TARGET_FPS };
    float _avgRenderTime { 0.0f }; // msec
    int _skippedRenderTimeSamples { 0 };
    bool _automaticLODAdjust = true;
};

#endif // hifi_LODManager_h
