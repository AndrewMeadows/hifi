//
//  LODManager.cpp
//  interface/src/LODManager.h
//
//  Created by Clement on 1/16/15.
//  Copyright 2015 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include <SettingHandle.h>
#include <OctreeUtils.h>
#include <Util.h>

#include "Application.h"
#include "ui/DialogsManager.h"
#include "InterfaceLogging.h"

#include "LODManager.h"

const float MIN_LOD_SCALE_FACTOR = 1.0f;

// In an attempt to maintain legacy behavior we set ABSOLUTE_MIN_ANGULAR_SIZE to be that of a
// 1x1x1 cube at 400 meters (which corresponds to about 0.00433 radians or 0.25 degrees).
const float SQRT_THREE = 1.7320508f;
const float ABSOLUTE_MIN_ANGULAR_SIZE = SQRT_THREE / 400.0f;


void LODManager::setMinVisibleAngularSize(float value) {
    if (value < ABSOLUTE_MIN_ANGULAR_SIZE) {
        value = ABSOLUTE_MIN_ANGULAR_SIZE;
    }
    // we store the ratio rather than the value
    _lodScaleFactor = value / ABSOLUTE_MIN_ANGULAR_SIZE;
}

float LODManager::getMinVisibleAngularSize() const {
    // return the angle value rather than the ratio
    return _lodScaleFactor * ABSOLUTE_MIN_ANGULAR_SIZE;
}

LODManager::LODManager() {
    // preset_avgRenderTime to be on target
    _avgRenderTime = 1.0f / _targetFPS;
}

const uint64_t LOD_ADJUST_PERIOD = 1 * USECS_PER_SECOND;

void LODManager::autoAdjustLOD(float batchTime, float engineRunTime, float deltaTimeSec) {

    float renderTime = batchTime + 2.0f; // msec
    float maxTime = glm::max(renderTime, engineRunTime);

    // skip the first several samples
    const int NUM_SAMPLES_TO_SKIP = 100;
    if (_skippedRenderTimeSamples < NUM_SAMPLES_TO_SKIP) {
        ++_skippedRenderTimeSamples;
        return;
    }

    // compute time-weighted running average
    const float BLEND_TIMESCALE = 0.3f; // sec
    float blend = BLEND_TIMESCALE / deltaTimeSec;
    if (blend > 1.0f) {
        blend = 1.0f;
    }
    _avgRenderTime = (1.0f - blend) * _avgRenderTime + blend * maxTime; // msec

    if (!_automaticLODAdjust) {
        _lodScaleFactor = MIN_LOD_SCALE_FACTOR;
        return;
    }

    uint64_t now = usecTimestampNow();
    if (now > _lodAdjustExpiry) {
        // check for LOD adjustment
        const float maxRenderTimeBudget = MSECS_PER_SECOND / _targetFPS; // msec
        const float minRenderTimeBudget = 0.5f * maxRenderTimeBudget;
        bool changed = false;
        if (_avgRenderTime > maxRenderTimeBudget) {
            // we're over budget --> decrease LOD (increase _lodScaleFactor)
            const float LOD_SCALE_FACTOR_INCREASE = 1.1f;
            _lodScaleFactor *= LOD_SCALE_FACTOR_INCREASE;
            changed = true;
        } else if (_avgRenderTime < minRenderTimeBudget && _lodScaleFactor > MIN_LOD_SCALE_FACTOR) {
            // we're well under budget --> increase LOD (decrease _lodScaleFactor)
            const float LOD_SCALE_FACTOR_DECREASE = 0.9f;
            _lodScaleFactor *= LOD_SCALE_FACTOR_DECREASE;
            if (_lodScaleFactor < MIN_LOD_SCALE_FACTOR) {
                _lodScaleFactor = MIN_LOD_SCALE_FACTOR;
            }
            changed = true;
        }

        if (changed) {
            auto lodToolsDialog = DependencyManager::get<DialogsManager>()->getLodToolsDialog();
            if (lodToolsDialog) {
                lodToolsDialog->reloadSliders();
            }
        }

        // set expiry for next adjustment
        _lodAdjustExpiry = now + LOD_ADJUST_PERIOD;
    }
}

void LODManager::resetLODAdjust() {
    _lodAdjustExpiry = usecTimestampNow() + LOD_ADJUST_PERIOD;
    _skippedRenderTimeSamples = 0;
    _lodScaleFactor = MIN_LOD_SCALE_FACTOR;
}

QString LODManager::getLODFeedbackText() {
    return QString("TODO: fix LODManager::getLODFeedback()");
/* TODO: andrew to implement this again
    // determine granularity feedback
    int boundaryLevelAdjust = getBoundaryLevelAdjust();
    QString granularityFeedback;
    
    switch (boundaryLevelAdjust) {
        case 0: {
            granularityFeedback = QString(".");
        } break;
        case 1: {
            granularityFeedback = QString(" at half of standard granularity.");
        } break;
        case 2: {
            granularityFeedback = QString(" at a third of standard granularity.");
        } break;
        default: {
            granularityFeedback = QString(" at 1/%1th of standard granularity.").arg(boundaryLevelAdjust + 1);
        } break;
    }
    
    // distance feedback
    float octreeSizeScale = getOctreeSizeScale();
    float relativeToDefault = octreeSizeScale / DEFAULT_OCTREE_SIZE_SCALE;
    int relativeToTwentyTwenty = 20 / relativeToDefault;

    QString result;
    if (relativeToDefault > 1.01f) {
        result = QString("20:%1 or %2 times further than average vision%3").arg(relativeToTwentyTwenty).arg(relativeToDefault,0,'f',2).arg(granularityFeedback);
    } else if (relativeToDefault > 0.99f) {
        result = QString("20:20 or the default distance for average vision%1").arg(granularityFeedback);
    } else if (relativeToDefault > 0.01f) {
        result = QString("20:%1 or %2 of default distance for average vision%3").arg(relativeToTwentyTwenty).arg(relativeToDefault,0,'f',3).arg(granularityFeedback);
    } else {
        result = QString("%2 of default distance for average vision%3").arg(relativeToDefault,0,'f',3).arg(granularityFeedback);
    }
    return result;
*/
}

bool LODManager::shouldRender(const RenderArgs* args, const AABox& bounds) {
    // should render when:
    //   apparentAngle > minApparentAngle
    //   (size / distance) > (lodScaleFactor * MIN_ANGLE)
    //   size > (distance * lodScaleFactor * MIN_ANGLE)
    //   size^2 > (distance * lodScaleFactor * MIN_ANGLE)^2
    float lodScaleFactor = DEFAULT_OCTREE_SIZE_SCALE / args->_sizeScale;
    float minAngle = lodScaleFactor * ABSOLUTE_MIN_ANGULAR_SIZE;
    float distance2 = glm::distance2(args->getViewFrustum().getPosition(), bounds.calcCenter());
    return glm::length2(bounds.getScale()) > distance2 * minAngle * minAngle;
};

void LODManager::loadSettings() {
    /* TODO: andrew to implement this again
    setDesktopLODDecreaseFPS(desktopLODDecreaseFPS.get());
    setHMDLODDecreaseFPS(hmdLODDecreaseFPS.get());
    */
}

void LODManager::saveSettings() {
    /* TODO: andrew to implement this again
    desktopLODDecreaseFPS.set(getDesktopLODDecreaseFPS());
    hmdLODDecreaseFPS.set(getHMDLODDecreaseFPS());
    */
}

void LODManager::setRenderMode(QString mode) {
    // TODO: make mode FPS's configurable as Settings rather than use hard-coded defaults
    if (mode == "HMD") {
        _targetFPS = DEFAULT_HMD_TARGET_FPS;
    } else if (mode == "Desktop") {
        _targetFPS = DEFAULT_DESKTOP_TARGET_FPS;
    } else {
        _targetFPS = DEFAULT_TARGET_FPS;
    }
    // preset_avgRenderTime to be on target
    _avgRenderTime = 1.0f / _targetFPS;
}
