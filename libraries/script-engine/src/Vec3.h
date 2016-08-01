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

/// Scriptable interface a Vec3ernion helper class object. Used exclusively in the JavaScript API
class Vec3 : public QObject {
    Q_OBJECT
    Q_PROPERTY(glm::vec3 UNIT_X READ UNIT_X CONSTANT)
    Q_PROPERTY(glm::vec3 UNIT_Y READ UNIT_Y CONSTANT)
    Q_PROPERTY(glm::vec3 UNIT_Z READ UNIT_Z CONSTANT)
    Q_PROPERTY(glm::vec3 UNIT_NEG_X READ UNIT_NEG_X CONSTANT)
    Q_PROPERTY(glm::vec3 UNIT_NEG_Y READ UNIT_NEG_Y CONSTANT)
    Q_PROPERTY(glm::vec3 UNIT_NEG_Z READ UNIT_NEG_Z CONSTANT)
    Q_PROPERTY(glm::vec3 UNIT_XY READ UNIT_XY CONSTANT)
    Q_PROPERTY(glm::vec3 UNIT_XZ READ UNIT_XZ CONSTANT)
    Q_PROPERTY(glm::vec3 UNIT_YZ READ UNIT_YZ CONSTANT)
    Q_PROPERTY(glm::vec3 UNIT_XYZ READ UNIT_XYZ CONSTANT)
    Q_PROPERTY(glm::vec3 FLOAT_MAX READ FLOAT_MAX CONSTANT)
    Q_PROPERTY(glm::vec3 FLOAT_MIN READ FLOAT_MIN CONSTANT)
    Q_PROPERTY(glm::vec3 ZERO READ ZERO CONSTANT)
    Q_PROPERTY(glm::vec3 ONE READ ONE CONSTANT)
    Q_PROPERTY(glm::vec3 TWO READ TWO CONSTANT)
    Q_PROPERTY(glm::vec3 HALF READ HALF CONSTANT)
    Q_PROPERTY(glm::vec3 RIGHT READ RIGHT CONSTANT)
    Q_PROPERTY(glm::vec3 UP READ UP CONSTANT)
    Q_PROPERTY(glm::vec3 FRONT READ FRONT CONSTANT)

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

private:
    const glm::vec3& UNIT_X() { return Vectors::UNIT_X; }
    const glm::vec3& UNIT_Y() { return Vectors::UNIT_Y; }
    const glm::vec3& UNIT_Z() { return Vectors::UNIT_Z; }
    const glm::vec3& UNIT_NEG_X() { return Vectors::UNIT_NEG_X; }
    const glm::vec3& UNIT_NEG_Y() { return Vectors::UNIT_NEG_Y; }
    const glm::vec3& UNIT_NEG_Z() { return Vectors::UNIT_NEG_Z; }
    const glm::vec3& UNIT_XY() { return Vectors::UNIT_XY; }
    const glm::vec3& UNIT_XZ() { return Vectors::UNIT_XZ; }
    const glm::vec3& UNIT_YZ() { return Vectors::UNIT_YZ; }
    const glm::vec3& UNIT_XYZ() { return Vectors::UNIT_XYZ; }
    const glm::vec3& FLOAT_MAX() { return Vectors::MAX; }
    const glm::vec3& FLOAT_MIN() { return Vectors::MIN; }
    const glm::vec3& ZERO() { return Vectors::ZERO; }
    const glm::vec3& ONE() { return Vectors::ONE; }
    const glm::vec3& TWO() { return Vectors::TWO; }
    const glm::vec3& HALF() { return Vectors::HALF; }
    const glm::vec3& RIGHT() { return Vectors::RIGHT; }
    const glm::vec3& UP() { return Vectors::UP; }
    const glm::vec3& FRONT() { return Vectors::FRONT; }
};

#endif // hifi_Vec3_h
