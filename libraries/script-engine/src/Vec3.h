//
//  Vec3.h
//  libraries/script-engine/src
//
//  Created by Brad Hefta-Gaub on 1/29/14.
//  Copyright 2014 High Fidelity, Inc.
//
//  Scriptable Vec3 class library.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#pragma once
#ifndef hifi_Vec3_h
#define hifi_Vec3_h

#include <QtCore/QObject>
#include <QtCore/QString>

#include "GLMHelpers.h"

/// Scriptable interface a Vec3 helper class object. Used exclusively in the JavaScript API
class Vec3 : public QObject {
    Q_OBJECT
    Q_PROPERTY(QVariant UNIT_X READ UNIT_X CONSTANT)
    Q_PROPERTY(QVariant UNIT_Y READ UNIT_Y CONSTANT)
    Q_PROPERTY(QVariant UNIT_Z READ UNIT_Z CONSTANT)
    Q_PROPERTY(QVariant UNIT_NEG_X READ UNIT_NEG_X CONSTANT)
    Q_PROPERTY(QVariant UNIT_NEG_Y READ UNIT_NEG_Y CONSTANT)
    Q_PROPERTY(QVariant UNIT_NEG_Z READ UNIT_NEG_Z CONSTANT)
    Q_PROPERTY(QVariant UNIT_XY READ UNIT_XY CONSTANT)
    Q_PROPERTY(QVariant UNIT_XZ READ UNIT_XZ CONSTANT)
    Q_PROPERTY(QVariant UNIT_YZ READ UNIT_YZ CONSTANT)
    Q_PROPERTY(QVariant UNIT_XYZ READ UNIT_XYZ CONSTANT)
    Q_PROPERTY(QVariant FLOAT_MAX READ FLOAT_MAX CONSTANT)
    Q_PROPERTY(QVariant FLOAT_MIN READ FLOAT_MIN CONSTANT)
    Q_PROPERTY(QVariant ZERO READ ZERO CONSTANT)
    Q_PROPERTY(QVariant ONE READ ONE CONSTANT)
    Q_PROPERTY(QVariant TWO READ TWO CONSTANT)
    Q_PROPERTY(QVariant HALF READ HALF CONSTANT)
    Q_PROPERTY(QVariant RIGHT READ RIGHT CONSTANT)
    Q_PROPERTY(QVariant UP READ UP CONSTANT)
    Q_PROPERTY(QVariant FRONT READ FRONT CONSTANT)

public slots:
    QVariant reflect(const QVariant& v1, const QVariant& v2);
    QVariant cross(const QVariant& v1, const QVariant& v2);
    float dot(const QVariant& v1, const QVariant& v2);
    QVariant multiply(const QVariant& v1, float f);
    QVariant multiply(float f, const QVariant& v1);
    QVariant multiplyVbyV(const QVariant& v1, const QVariant& v2);
    QVariant multiplyQbyV(const QVariant& q, const QVariant& v);
    QVariant sum(const QVariant& v1, const QVariant& v2);
    QVariant subtract(const QVariant& v1, const QVariant& v2);
    float length(const QVariant& v);
    float distance(const QVariant& v1, const QVariant& v2);
    float orientedAngle(const QVariant& v1, const QVariant& v2, const QVariant& v3);
    QVariant normalize(const QVariant& v);
    QVariant mix(const QVariant& v1, const QVariant& v2, float m);
    void print(const QString& label, const QVariant& v);
    bool equal(const QVariant& v1, const QVariant& v2);
    bool withinEpsilon(const QVariant& v1, const QVariant& v2, float epsilon);
    QVariant toSpherical(const QVariant& v);
    QVariant fromSpherical(const QVariant& polar);
    QVariant fromSpherical(float elevation, float azimuth);

    // for QJSEngine so it can access things like MyAvatar.position
    glm::vec3 fromObject(const QVariant& v) { return vec3FromVariant(v); }
    QVariant toObject(const glm::vec3& v) { return vec3toVariant(v); }

private:
    const QVariant& UNIT_X() { return vec3toVariant(Vectors::UNIT_X); }
    const QVariant& UNIT_Y() { return vec3toVariant(Vectors::UNIT_Y); }
    const QVariant& UNIT_Z() { return vec3toVariant(Vectors::UNIT_Z); }
    const QVariant& UNIT_NEG_X() { return vec3toVariant(Vectors::UNIT_NEG_X); }
    const QVariant& UNIT_NEG_Y() { return vec3toVariant(Vectors::UNIT_NEG_Y); }
    const QVariant& UNIT_NEG_Z() { return vec3toVariant(Vectors::UNIT_NEG_Z); }
    const QVariant& UNIT_XY() { return vec3toVariant(Vectors::UNIT_XY); }
    const QVariant& UNIT_XZ() { return vec3toVariant(Vectors::UNIT_XZ); }
    const QVariant& UNIT_YZ() { return vec3toVariant(Vectors::UNIT_YZ); }
    const QVariant& UNIT_XYZ() { return vec3toVariant(Vectors::UNIT_XYZ); }
    const QVariant& FLOAT_MAX() { return vec3toVariant(Vectors::MAX); }
    const QVariant& FLOAT_MIN() { return vec3toVariant(Vectors::MIN); }
    const QVariant& ZERO() { return vec3toVariant(Vectors::ZERO); }
    const QVariant& ONE() { return vec3toVariant(Vectors::ONE); }
    const QVariant& TWO() { return vec3toVariant(Vectors::TWO); }
    const QVariant& HALF() { return vec3toVariant(Vectors::HALF); }
    const QVariant& RIGHT() { return vec3toVariant(Vectors::RIGHT); }
    const QVariant& UP() { return vec3toVariant(Vectors::UP); }
    const QVariant& FRONT() { return vec3toVariant(Vectors::FRONT); }
};

#endif // hifi_Vec3_h
