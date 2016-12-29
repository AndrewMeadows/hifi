//
//  AnimPose.h
//
//  Created by Anthony J. Thibault on 10/14/15.
//  Copyright (c) 2015 High Fidelity, Inc. All rights reserved.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_AnimPose
#define hifi_AnimPose

#include <QtGlobal>
#include <QDebug>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

struct AnimPose {
    AnimPose() {}
    explicit AnimPose(const glm::mat4& mat);
    AnimPose(const glm::vec3& scaleIn, const glm::quat& rotIn, const glm::vec3& transIn) : _scale(scaleIn), _rot(rotIn), _trans(transIn) {}
    static const AnimPose identity;

    glm::vec3 xformPoint(const glm::vec3& rhs) const;
    glm::vec3 xformVector(const glm::vec3& rhs) const;  // really slow

    glm::vec3 operator*(const glm::vec3& rhs) const; // same as xformPoint
    AnimPose operator*(const AnimPose& rhs) const;

    AnimPose inverse() const;
    AnimPose mirror() const;
    operator glm::mat4() const;

    glm::vec3 getTranslation() const { return _trans; }
    glm::quat getRotation() const { return _rot; }
    glm::vec3 getScale() const { return _scale; }

    void setTranslation(const glm::vec3& t) { _trans = t; }
    void setRotation(const glm::quat& q) { _rot = q; }
    void setScale(const glm::vec3& s) { _scale = s; }

private:
    glm::vec3 _scale;
    glm::quat _rot;
    glm::vec3 _trans;
};

inline QDebug operator<<(QDebug debug, const AnimPose& pose) {
    glm::vec3 trans = pose.getTranslation();
    glm::quat rot = pose.getRotation();
    glm::vec3 scale = pose.getScale();
    debug << "AnimPose, trans = (" << trans.x << trans.y << trans.z << "), "
        << "rot = (" << rot.x << rot.y << rot.z << rot.w << "), "
        << "scale = (" << scale.x << scale.y << scale.z << ")";
    return debug;
}

using AnimPoseVec = std::vector<AnimPose>;

#endif
