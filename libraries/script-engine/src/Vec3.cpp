//
//  Vec3.cpp
//  libraries/script-engine/src
//
//  Created by Brad Hefta-Gaub on 1/29/14.
//  Copyright 2014 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include <glm/gtx/vector_angle.hpp>

#include <QDebug>

#include <GLMHelpers.h>

#include "RegisteredMetaTypes.h"
#include "ScriptEngineLogging.h"
#include "NumericalConstants.h"
#include "Vec3.h"


QVariant Vec3::reflect(const QVariant& v1, const QVariant& v2) {
    return vec3toVariant(glm::reflect(vec3FromVariant(v1), vec3FromVariant(v2)));
}

QVariant Vec3::cross(const QVariant& v1, const QVariant& v2) {
    return vec3toVariant(glm::cross(vec3FromVariant(v1), vec3FromVariant(v2)));
}

float Vec3::dot(const QVariant& v1, const QVariant& v2) {
    return glm::dot(vec3FromVariant(v1), vec3FromVariant(v2));
}

QVariant Vec3::multiply(const QVariant& v1, float f) {
    return vec3toVariant(vec3FromVariant(v1) * f);
}

QVariant Vec3::multiply(float f, const QVariant& v1) {
    return vec3toVariant(vec3FromVariant(v1) * f);
}

QVariant Vec3::multiplyVbyV(const QVariant& v1, const QVariant& v2) {
    return vec3toVariant(vec3FromVariant(v1) * vec3FromVariant(v2));
}

QVariant Vec3::multiplyQbyV(const QVariant& q, const QVariant& v) {
    return vec3toVariant(quatFromVariant(q) * vec3FromVariant(v));
}

QVariant Vec3::sum(const QVariant& v1, const QVariant& v2) {
    return vec3toVariant(vec3FromVariant(v1) + vec3FromVariant(v2));
}

QVariant Vec3::subtract(const QVariant& v1, const QVariant& v2) {
    return vec3toVariant(vec3FromVariant(v1) - vec3FromVariant(v2));
}

float Vec3::length(const QVariant& v) {
    return glm::length(vec3FromVariant(v));
}

float Vec3::distance(const QVariant& v1, const QVariant& v2) {
    return glm::distance(vec3FromVariant(v1), vec3FromVariant(v2));
}

float Vec3::orientedAngle(const QVariant& v1, const QVariant& v2, const QVariant& v3) {
    return glm::orientedAngle(
        glm::normalize(vec3FromVariant(v1)),
        glm::normalize(vec3FromVariant(v2)),
        glm::normalize(vec3FromVariant(v3))
    );
}

QVariant Vec3::normalize(const QVariant& v) {
    return vec3toVariant(glm::normalize(vec3FromVariant(v)));
}

QVariant Vec3::mix(const QVariant& v1, const QVariant& v2, float m) {
    return vec3toVariant(glm::mix(vec3FromVariant(v1), vec3FromVariant(v2), m));
}

void Vec3::print(const QString& label, const QVariant& variant) {
    auto v = vec3FromVariant(variant);
    qCDebug(scriptengine) << qPrintable(label) << v.x << "," << v.y << "," << v.z;
}

bool Vec3::equal(const QVariant& v1, const QVariant& v2) {
    return vec3FromVariant(v1) == vec3FromVariant(v2);
}

bool Vec3::withinEpsilon(const QVariant& v1, const QVariant& v2, float epsilon) {
    float distanceSquared = glm::length2(vec3FromVariant(v1) - vec3FromVariant(v2));
    return (epsilon*epsilon) >= distanceSquared;
}

QVariant Vec3::toSpherical(const QVariant& variant) {
    float radius = length(variant);
    auto v = vec3FromVariant(variant);
    if (glm::abs(radius) < EPSILON) {
        return vec3toVariant(glm::vec3(0.0f, 0.0f, 0.0f));
    }
    
    glm::vec3 u = v / radius;
    
    float elevation, azimuth;
    
    elevation = glm::asin(-u.y);
    azimuth = atan2(v.x, v.z);
    
    // Round off small decimal values
    if (glm::abs(elevation) < EPSILON) {
        elevation = 0.0f;
    }
    if (glm::abs(azimuth) < EPSILON) {
        azimuth = 0.0f;
    }

    return vec3toVariant(glm::vec3(elevation, azimuth, radius));
}

QVariant Vec3::fromSpherical(const QVariant& variant) {
    auto polar = vec3FromVariant(variant);
    float x = glm::cos(polar.x) * glm::sin(polar.y);
    float y = glm::sin(-polar.x);
    float z = glm::cos(polar.x) * glm::cos(polar.y);

    // Round small values to 0
    if (glm::abs(x) < EPSILON) {
        x = 0.0f;
    }
    if (glm::abs(y) < EPSILON) {
        y = 0.0f;
    }
    if (glm::abs(z) < EPSILON) {
        z = 0.0f;
    }

    return vec3toVariant(polar.z * glm::vec3(x, y, z));
}

QVariant Vec3::fromSpherical(float elevation, float azimuth) {
    return fromSpherical(vec3toVariant(glm::vec3(elevation, azimuth, 1.0f)));
}
