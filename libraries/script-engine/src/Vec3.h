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
    Q_PROPERTY(QVariant UNIT_X READ UNIT_X_GETTER CONSTANT)
    Q_PROPERTY(QVariant UNIT_Y READ UNIT_Y_GETTER CONSTANT)
    Q_PROPERTY(QVariant UNIT_Z READ UNIT_Z_GETTER CONSTANT)
    Q_PROPERTY(QVariant UNIT_NEG_X READ UNIT_NEG_X_GETTER CONSTANT)
    Q_PROPERTY(QVariant UNIT_NEG_Y READ UNIT_NEG_Y_GETTER CONSTANT)
    Q_PROPERTY(QVariant UNIT_NEG_Z READ UNIT_NEG_Z_GETTER CONSTANT)
    Q_PROPERTY(QVariant UNIT_XY READ UNIT_XY_GETTER CONSTANT)
    Q_PROPERTY(QVariant UNIT_XZ READ UNIT_XZ_GETTER CONSTANT)
    Q_PROPERTY(QVariant UNIT_YZ READ UNIT_YZ_GETTER CONSTANT)
    Q_PROPERTY(QVariant UNIT_XYZ READ UNIT_XYZ_GETTER CONSTANT)
    Q_PROPERTY(QVariant FLOAT_MAX READ FLOAT_MAX_GETTER CONSTANT)
    Q_PROPERTY(QVariant FLOAT_MIN READ FLOAT_MIN_GETTER CONSTANT)
    Q_PROPERTY(QVariant ZERO READ ZERO_GETTER CONSTANT)
    Q_PROPERTY(QVariant ONE READ ONE_GETTER CONSTANT)
    Q_PROPERTY(QVariant TWO READ TWO_GETTER CONSTANT)
    Q_PROPERTY(QVariant HALF READ HALF_GETTER CONSTANT)
    Q_PROPERTY(QVariant RIGHT READ RIGHT_GETTER CONSTANT)
    Q_PROPERTY(QVariant UP READ UP_GETTER CONSTANT)
    Q_PROPERTY(QVariant FRONT READ FRONT_GETTER CONSTANT)

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
    QVariant UNIT_X{ vec3toVariant(Vectors::UNIT_X) };
    QVariant UNIT_Y{ vec3toVariant(Vectors::UNIT_Y) };
    QVariant UNIT_Z{ vec3toVariant(Vectors::UNIT_Z) };
    QVariant UNIT_NEG_X{ vec3toVariant(Vectors::UNIT_NEG_X) };
    QVariant UNIT_NEG_Y{ vec3toVariant(Vectors::UNIT_NEG_Y) };
    QVariant UNIT_NEG_Z{ vec3toVariant(Vectors::UNIT_NEG_Z) };
    QVariant UNIT_XY{ vec3toVariant(Vectors::UNIT_XY) };
    QVariant UNIT_XZ{ vec3toVariant(Vectors::UNIT_XZ) };
    QVariant UNIT_YZ{ vec3toVariant(Vectors::UNIT_YZ) };
    QVariant UNIT_XYZ{ vec3toVariant(Vectors::UNIT_XYZ) };
    QVariant FLOAT_MAX{ vec3toVariant(Vectors::MAX) };
    QVariant FLOAT_MIN{ vec3toVariant(Vectors::MIN) };
    QVariant ZERO{ vec3toVariant(Vectors::ZERO) };
    QVariant ONE{ vec3toVariant(Vectors::ONE) };
    QVariant TWO{ vec3toVariant(Vectors::TWO) };
    QVariant HALF{ vec3toVariant(Vectors::HALF) };
    QVariant RIGHT{ vec3toVariant(Vectors::RIGHT) };
    QVariant UP{ vec3toVariant(Vectors::UP) };
    QVariant FRONT{ vec3toVariant(Vectors::FRONT) };

    const QVariant& UNIT_X_GETTER() { return UNIT_X; }
    const QVariant& UNIT_Y_GETTER() { return UNIT_Y; }
    const QVariant& UNIT_Z_GETTER() { return UNIT_Z; }
    const QVariant& UNIT_NEG_X_GETTER() { return UNIT_NEG_X; }
    const QVariant& UNIT_NEG_Y_GETTER() { return UNIT_NEG_Y; }
    const QVariant& UNIT_NEG_Z_GETTER() { return UNIT_NEG_Z; }
    const QVariant& UNIT_XY_GETTER() { return UNIT_XY; }
    const QVariant& UNIT_XZ_GETTER() { return UNIT_XZ; }
    const QVariant& UNIT_YZ_GETTER() { return UNIT_YZ; }
    const QVariant& UNIT_XYZ_GETTER() { return UNIT_XYZ; }
    const QVariant& FLOAT_MAX_GETTER() { return FLOAT_MAX; }
    const QVariant& FLOAT_MIN_GETTER() { return FLOAT_MIN; }
    const QVariant& ZERO_GETTER() { return ZERO; }
    const QVariant& ONE_GETTER() { return ONE; }
    const QVariant& TWO_GETTER() { return TWO; }
    const QVariant& HALF_GETTER() { return HALF; }
    const QVariant& RIGHT_GETTER() { return RIGHT; }
    const QVariant& UP_GETTER() { return UP; }
    const QVariant& FRONT_GETTER() { return FRONT; }
};

#endif // hifi_Vec3_h
