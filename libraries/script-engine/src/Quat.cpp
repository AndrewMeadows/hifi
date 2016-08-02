//
//  Quat.cpp
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

#include <OctreeConstants.h>
#include <GLMHelpers.h>

#include "RegisteredMetaTypes.h"
#include "ScriptEngineLogging.h"
#include "Quat.h"

QVariant Quat::normalize(const QVariant& q) {
    return quatToVariant(glm::normalize(quatFromVariant(q)));
}

QVariant Quat::conjugate(const QVariant& q) {
    return quatToVariant(glm::conjugate(quatFromVariant(q)));
}

QVariant Quat::rotationBetween(const QVariant& v1, const QVariant& v2) {
    return quatToVariant(::rotationBetween(vec3FromVariant(v1), vec3FromVariant(v2)));
}

QVariant Quat::lookAt(const QVariant& eye, const QVariant& center, const QVariant& up) {
    // OpenGL view matrix is inverse of our camera matrix.
    return quatToVariant(glm::inverse(glm::quat_cast(glm::lookAt(vec3FromVariant(eye), vec3FromVariant(center), 
                                                                 vec3FromVariant(up)))));
}

QVariant Quat::lookAtSimple(const QVariant& eye, const QVariant& center) {
    auto dir = glm::normalize(vec3FromVariant(center) - vec3FromVariant(eye));
    // if the direction is nearly aligned with the Y axis, then use the X axis for 'up'
    if (dir.x < 0.001f && dir.z < 0.001f) {
        return lookAt(eye, center, vec3toVariant(Vectors::UNIT_X));
    }
    return lookAt(eye, center, vec3toVariant(Vectors::UNIT_Y));
}

QVariant Quat::multiply(const QVariant& q1, const QVariant& q2) { 
    return quatToVariant(quatFromVariant(q1) * quatFromVariant(q2));
}

QVariant Quat::fromVec3Degrees(const QVariant& eulerAngles) { 
    return quatToVariant(glm::quat(glm::radians(vec3FromVariant(eulerAngles))));
}

QVariant Quat::fromVec3Radians(const QVariant& eulerAngles) { 
    return quatToVariant(glm::quat(vec3FromVariant(eulerAngles)));
}

QVariant Quat::fromPitchYawRollDegrees(float pitch, float yaw, float roll) { 
    return quatToVariant(glm::quat(glm::radians(glm::vec3(pitch, yaw, roll))));
}

QVariant Quat::fromPitchYawRollRadians(float pitch, float yaw, float roll) { 
    return quatToVariant(glm::quat(glm::vec3(pitch, yaw, roll)));
}

QVariant Quat::inverse(const QVariant& q) {
    return quatToVariant(glm::inverse(quatFromVariant(q)));
}

QVariant Quat::getFront(const QVariant& orientation) {
    return vec3toVariant(quatFromVariant(orientation) * Vectors::FRONT);
}

QVariant Quat::getRight(const QVariant& orientation) {
    return vec3toVariant(quatFromVariant(orientation) * Vectors::RIGHT);
}

QVariant Quat::getUp(const QVariant& orientation) {
    return vec3toVariant(quatFromVariant(orientation) * Vectors::UP);
}

QVariant Quat::safeEulerAngles(const QVariant& orientation) {
    return vec3toVariant(glm::degrees(::safeEulerAngles(quatFromVariant(orientation))));
}

QVariant Quat::angleAxis(float angle, const QVariant& v) {
    return quatToVariant(glm::angleAxis(glm::radians(angle), vec3FromVariant(v)));
}

QVariant Quat::axis(const QVariant& orientation) {
    return vec3toVariant(glm::axis(quatFromVariant(orientation)));
}

float Quat::angle(const QVariant& orientation) {
    return glm::angle(quatFromVariant(orientation));
}

QVariant Quat::mix(const QVariant& q1, const QVariant& q2, float alpha) {
    return quatToVariant(safeMix(quatFromVariant(q1), quatFromVariant(q2), alpha));
}

/// Spherical Linear Interpolation
QVariant Quat::slerp(const QVariant& q1, const QVariant& q2, float alpha) {
    return quatToVariant(glm::slerp(quatFromVariant(q1), quatFromVariant(q2), alpha));
}

// Spherical Quadratic Interpolation
QVariant Quat::squad(const QVariant& q1, const QVariant& q2, const QVariant& s1, const QVariant& s2, float h) {
    return quatToVariant(glm::squad(quatFromVariant(q1), quatFromVariant(q2), quatFromVariant(s1), quatFromVariant(s2), h));
}

float Quat::dot(const QVariant& q1, const QVariant& q2) {
    return glm::dot(quatFromVariant(q1), quatFromVariant(q2));
}

void Quat::print(const QString& label, const QVariant& variant) {
    auto q = quatFromVariant(variant);
    qCDebug(scriptengine) << qPrintable(label) << q.x << "," << q.y << "," << q.z << "," << q.w;
}

bool Quat::equal(const QVariant& q1, const QVariant& q2) {
    return quatFromVariant(q1) == quatFromVariant(q2);
}

