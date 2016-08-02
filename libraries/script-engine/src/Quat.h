//
//  Quat.h
//  libraries/script-engine/src
//
//  Created by Brad Hefta-Gaub on 1/29/14.
//  Copyright 2014 High Fidelity, Inc.
//
//  Scriptable Quaternion class library.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_Quat_h
#define hifi_Quat_h

#include <glm/gtc/quaternion.hpp>

#include <QObject>
#include <QString>

/// Scriptable interface a Quaternion helper class object. Used exclusively in the JavaScript API
class Quat : public QObject {
    Q_OBJECT

public slots:
    QVariant multiply(const QVariant& q1, const QVariant& q2);
    QVariant normalize(const QVariant& q);
    QVariant conjugate(const QVariant& q);
    QVariant lookAt(const QVariant& eye, const QVariant& center, const QVariant& up);
    QVariant lookAtSimple(const QVariant& eye, const QVariant& center);
    QVariant rotationBetween(const QVariant& v1, const QVariant& v2);
    QVariant fromVec3Degrees(const QVariant& vec3); // degrees
    QVariant fromVec3Radians(const QVariant& vec3); // radians
    QVariant fromPitchYawRollDegrees(float pitch, float yaw, float roll); // degrees
    QVariant fromPitchYawRollRadians(float pitch, float yaw, float roll); // radians
    QVariant inverse(const QVariant& q);
    QVariant getFront(const QVariant& orientation);
    QVariant getRight(const QVariant& orientation);
    QVariant getUp(const QVariant& orientation);
    QVariant safeEulerAngles(const QVariant& orientation); // degrees
    QVariant angleAxis(float angle, const QVariant& v);   // degrees
    QVariant axis(const QVariant& orientation);
    float angle(const QVariant& orientation);
    QVariant mix(const QVariant& q1, const QVariant& q2, float alpha);
    QVariant slerp(const QVariant& q1, const QVariant& q2, float alpha);
    QVariant squad(const QVariant& q1, const QVariant& q2, const QVariant& s1, const QVariant& s2, float h);
    float dot(const QVariant& q1, const QVariant& q2);
    void print(const QString& label, const QVariant& q);
    bool equal(const QVariant& q1, const QVariant& q2);

    // for QJSEngine so it can access things like MyAvatar.orientation
    glm::quat fromObject(const QVariant& v) { return quatFromVariant(v); }
    QVariant toObject(const glm::quat& v) { return quatToVariant(v); }
};

#endif // hifi_Quat_h
