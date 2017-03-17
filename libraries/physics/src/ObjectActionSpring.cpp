//
//  ObjectActionSpring.cpp
//  libraries/physics/src
//
//  Created by Seth Alves 2015-6-5
//  Copyright 2015 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "QVariantGLM.h"

#include "ObjectActionSpring.h"

#include "PhysicsLogging.h"

const float SPRING_MAX_SPEED = 10.0f;

const uint16_t ObjectActionSpring::springVersion = 1;


ObjectActionSpring::ObjectActionSpring(const QUuid& id, EntityItemPointer ownerEntity) :
    ObjectAction(ACTION_TYPE_SPRING, id, ownerEntity),
    _positionalTarget(glm::vec3(0.0f)),
    _desiredPositionalTarget(glm::vec3(0.0f)),
    _linearTimeScale(FLT_MAX),
    _positionalTargetSet(true),
    _rotationalTarget(glm::quat()),
    _desiredRotationalTarget(glm::quat()),
    _angularTimeScale(FLT_MAX),
    _rotationalTargetSet(true) {
    #if WANT_DEBUG
    qCDebug(physics) << "ObjectActionSpring::ObjectActionSpring";
    #endif
}

ObjectActionSpring::~ObjectActionSpring() {
    #if WANT_DEBUG
    qCDebug(physics) << "ObjectActionSpring::~ObjectActionSpring";
    #endif
}

bool ObjectActionSpring::getTarget(float deltaTimeStep, glm::quat& rotation, glm::vec3& position,
                                   glm::vec3& linearVelocity, glm::vec3& angularVelocity) {
    rotation = _desiredRotationalTarget;
    position = _desiredPositionalTarget;
    linearVelocity = glm::vec3();
    angularVelocity = glm::vec3();
    return true;
}

bool ObjectActionSpring::prepareForSpringUpdate(btScalar deltaTimeStep) {
    auto ownerEntity = _ownerEntity.lock();
    if (!ownerEntity) {
        return false;
    }

    glm::quat rotation;
    glm::vec3 position;
    glm::vec3 linearVelocity;
    glm::vec3 angularVelocity;

    bool valid = false;
    int springCount = 0;

    QList<EntityActionPointer> springDerivedActions;
    springDerivedActions.append(ownerEntity->getActionsOfType(ACTION_TYPE_SPRING));
    springDerivedActions.append(ownerEntity->getActionsOfType(ACTION_TYPE_HOLD));

    foreach (EntityActionPointer action, springDerivedActions) {
        std::shared_ptr<ObjectActionSpring> springAction = std::static_pointer_cast<ObjectActionSpring>(action);
        glm::quat rotationForAction;
        glm::vec3 positionForAction;
        glm::vec3 linearVelocityForAction, angularVelocityForAction;
        bool success = springAction->getTarget(deltaTimeStep, rotationForAction,
                                               positionForAction,  linearVelocityForAction,
                                               angularVelocityForAction);
        if (success) {
            springCount ++;
            if (springAction.get() == this) {
                // only use the rotation for this action
                valid = true;
                rotation = rotationForAction;
            }

            position += positionForAction;
            linearVelocity += linearVelocityForAction;
            angularVelocity += angularVelocityForAction;
        }
    }

    if (valid && springCount > 0) {
        position /= springCount;
        linearVelocity /= springCount;
        angularVelocity /= springCount;

        withWriteLock([&]{
            _positionalTarget = position;
            _rotationalTarget = rotation;
            _linearVelocityTarget = linearVelocity;
            _angularVelocityTarget = angularVelocity;
            _positionalTargetSet = true;
            _rotationalTargetSet = true;
            _active = true;
        });
    }

    return valid;
}


void ObjectActionSpring::updateActionWorker(btScalar deltaTimeStep) {
    if (!prepareForSpringUpdate(deltaTimeStep)) {
        return;
    }

    withReadLock([&]{
        auto ownerEntity = _ownerEntity.lock();
        if (!ownerEntity) {
            return;
        }

        void* physicsInfo = ownerEntity->getPhysicsInfo();
        if (!physicsInfo) {
            return;
        }
        ObjectMotionState* motionState = static_cast<ObjectMotionState*>(physicsInfo);
        btRigidBody* rigidBody = motionState->getRigidBody();
        if (!rigidBody) {
            qCDebug(physics) << "ObjectActionSpring::updateActionWorker no rigidBody";
            return;
        }


        /*JCK
        equations of motion for a mass m attached to a spring with damping constant b and spring constant k
        dt = deltaTimeStep
        x_{N+1} = xN + vN*dt
        x_{N+1} is the position at the (N+1)th iteration
        x_N is the position at the Nth iteration
        
        v_{N+1} = vN + (1/m)*(-b*vN - k*xN)*dt
        v_{N+1} is the velocity at the (N+1)th iteration
        v_N is the velocity at the Nth iteration

        */

        btScalar k = 2.0; //spring constant
        //I don't know where else k is set, so I'll set it here
        btVector3 targetVelocity(0.0f, 0.0f, 0.0f);
        btVector3 newPosition = rigidBody->getCenterOfMassPosition() + rigidBody->getLinearVelocity()*deltaTimeStep;
        targetVelocity = rigidBody->getLinearVelocity() + (1/m_mass)*(-rigidBody->etLinearDamping()*m_currentVelocity - k*rigidBody->getCenterOfMassPosition())*deltaTimeStep;
        //I see m_mass for rigidBody is set in btRigidBodyConstructionInfo
        //But I wasn't sure how to return the mass for a rigidBody
        rigidBody->setLinearVelocity(targetVelocity);

        //I know this my code is flawed, but I wanted to produce some code quickly
        //so I decided to just go with this

        //end JCK
    });
}

const float MIN_TIMESCALE = 0.1f;


bool ObjectActionSpring::updateArguments(QVariantMap arguments) {
    glm::vec3 positionalTarget;
    float linearTimeScale;
    glm::quat rotationalTarget;
    float angularTimeScale;

    bool needUpdate = false;
    bool somethingChanged = ObjectAction::updateArguments(arguments);
    withReadLock([&]{
        // targets are required, spring-constants are optional
        bool ok = true;
        positionalTarget = EntityActionInterface::extractVec3Argument("spring action", arguments, "targetPosition", ok, false);
        if (!ok) {
            positionalTarget = _desiredPositionalTarget;
        }
        ok = true;
        linearTimeScale = EntityActionInterface::extractFloatArgument("spring action", arguments, "linearTimeScale", ok, false);
        if (!ok || linearTimeScale <= 0.0f) {
            linearTimeScale = _linearTimeScale;
        }

        ok = true;
        rotationalTarget = EntityActionInterface::extractQuatArgument("spring action", arguments, "targetRotation", ok, false);
        if (!ok) {
            rotationalTarget = _desiredRotationalTarget;
        }

        ok = true;
        angularTimeScale =
            EntityActionInterface::extractFloatArgument("spring action", arguments, "angularTimeScale", ok, false);
        if (!ok) {
            angularTimeScale = _angularTimeScale;
        }

        if (somethingChanged ||
            positionalTarget != _desiredPositionalTarget ||
            linearTimeScale != _linearTimeScale ||
            rotationalTarget != _desiredRotationalTarget ||
            angularTimeScale != _angularTimeScale) {
            // something changed
            needUpdate = true;
        }
    });

    if (needUpdate) {
        withWriteLock([&] {
            _desiredPositionalTarget = positionalTarget;
            _linearTimeScale = glm::max(MIN_TIMESCALE, glm::abs(linearTimeScale));
            _desiredRotationalTarget = rotationalTarget;
            _angularTimeScale = glm::max(MIN_TIMESCALE, glm::abs(angularTimeScale));
            _active = true;

            auto ownerEntity = _ownerEntity.lock();
            if (ownerEntity) {
                ownerEntity->setActionDataDirty(true);
                ownerEntity->setActionDataNeedsTransmit(true);
            }
        });
        activateBody();
    }

    return true;
}

QVariantMap ObjectActionSpring::getArguments() {
    QVariantMap arguments = ObjectAction::getArguments();
    withReadLock([&] {
        arguments["linearTimeScale"] = _linearTimeScale;
        arguments["targetPosition"] = glmToQMap(_desiredPositionalTarget);

        arguments["targetRotation"] = glmToQMap(_desiredRotationalTarget);
        arguments["angularTimeScale"] = _angularTimeScale;
    });
    return arguments;
}

QByteArray ObjectActionSpring::serialize() const {
    QByteArray serializedActionArguments;
    QDataStream dataStream(&serializedActionArguments, QIODevice::WriteOnly);

    dataStream << ACTION_TYPE_SPRING;
    dataStream << getID();
    dataStream << ObjectActionSpring::springVersion;

    withReadLock([&] {
        dataStream << _desiredPositionalTarget;
        dataStream << _linearTimeScale;
        dataStream << _positionalTargetSet;
        dataStream << _desiredRotationalTarget;
        dataStream << _angularTimeScale;
        dataStream << _rotationalTargetSet;
        dataStream << localTimeToServerTime(_expires);
        dataStream << _tag;
    });

    return serializedActionArguments;
}

void ObjectActionSpring::deserialize(QByteArray serializedArguments) {
    QDataStream dataStream(serializedArguments);

    EntityActionType type;
    dataStream >> type;
    assert(type == getType());

    QUuid id;
    dataStream >> id;
    assert(id == getID());

    uint16_t serializationVersion;
    dataStream >> serializationVersion;
    if (serializationVersion != ObjectActionSpring::springVersion) {
        assert(false);
        return;
    }

    withWriteLock([&] {
        dataStream >> _desiredPositionalTarget;
        dataStream >> _linearTimeScale;
        dataStream >> _positionalTargetSet;

        dataStream >> _desiredRotationalTarget;
        dataStream >> _angularTimeScale;
        dataStream >> _rotationalTargetSet;

        quint64 serverExpires;
        dataStream >> serverExpires;
        _expires = serverTimeToLocalTime(serverExpires);

        dataStream >> _tag;

        _active = true;
    });
}
