//
//  EntityItemPropertiesMacros.h
//  libraries/entities/src
//
//  Created by Brad Hefta-Gaub on 9/10/14.
//  Copyright 2014 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//


#ifndef hifi_EntityItemPropertiesMacros_h
#define hifi_EntityItemPropertiesMacros_h

#include <QDateTime>

#include "EntityItemID.h"
#include <RegisteredMetaTypes.h>

#define APPEND_ENTITY_PROPERTY(P,V) \
        if (requestedProperties.getHasProperty(P)) {                \
            LevelDetails propertyLevel = packetData->startLevel();  \
            successPropertyFits = packetData->appendValue(V);       \
            if (successPropertyFits) {                              \
                propertyFlags |= P;                                 \
                propertiesDidntFit -= P;                            \
                propertyCount++;                                    \
                packetData->endLevel(propertyLevel);                \
            } else {                                                \
                packetData->discardLevel(propertyLevel);            \
                appendState = OctreeElement::PARTIAL;               \
            }                                                       \
        } else {                                                    \
            propertiesDidntFit -= P;                                \
        }

#define READ_ENTITY_PROPERTY(P,T,S)                                                \
        if (propertyFlags.getHasProperty(P)) {                                     \
            T fromBuffer;                                                          \
            int bytes = OctreePacketData::unpackDataFromBytes(dataAt, fromBuffer); \
            dataAt += bytes;                                                       \
            bytesRead += bytes;                                                    \
            if (overwriteLocalData) {                                              \
                S(fromBuffer);                                                     \
            }                                                                      \
            somethingChanged = true;                                               \
        }

#define SKIP_ENTITY_PROPERTY(P,T)                                                  \
        if (propertyFlags.getHasProperty(P)) {                                     \
            T fromBuffer;                                                          \
            int bytes = OctreePacketData::unpackDataFromBytes(dataAt, fromBuffer); \
            dataAt += bytes;                                                       \
            bytesRead += bytes;                                                    \
        }

#define DECODE_GROUP_PROPERTY_HAS_CHANGED(P,N) \
        if (propertyFlags.getHasProperty(P)) {  \
            set##N##Changed(true); \
        }


#define READ_ENTITY_PROPERTY_TO_PROPERTIES(P,T,O)                                  \
        if (propertyFlags.getHasProperty(P)) {                                     \
            T fromBuffer;                                                          \
            int bytes = OctreePacketData::unpackDataFromBytes(dataAt, fromBuffer); \
            dataAt += bytes;                                                       \
            processedBytes += bytes;                                               \
            properties.O(fromBuffer);                                              \
        }

#define SET_ENTITY_PROPERTY_FROM_PROPERTIES(P,M)    \
    if (properties._##P##Changed) {    \
        M(properties._##P);                         \
        somethingChanged = true;                    \
    }

#define SET_ENTITY_GROUP_PROPERTY_FROM_PROPERTIES(G,P,p,M)  \
    if (properties.get##G().p##Changed()) {                 \
        M(properties.get##G().get##P());                    \
        somethingChanged = true;                            \
    }

#define SET_ENTITY_PROPERTY_FROM_PROPERTIES_GETTER(C,G,S)    \
    if (properties.C()) {    \
        S(properties.G());                         \
        somethingChanged = true;                    \
    }

#define COPY_ENTITY_PROPERTY_TO_PROPERTIES(P,M) \
    properties._##P = M();                      \
    properties._##P##Changed = false;

#define COPY_ENTITY_GROUP_PROPERTY_TO_PROPERTIES(G,P,M)  \
    properties.get##G().set##P(M());                     \
    properties.get##G().set##P##Changed(false);

#define CHECK_PROPERTY_CHANGE(P,M) \
    if (_##M##Changed) {           \
        changedProperties += P;    \
    }


typedef glm::vec3 glmVec3;
typedef glm::quat glmQuat;
typedef QVector<glm::vec3> qVectorVec3;
typedef QVector<glm::quat> qVectorQuat;
typedef QVector<bool> qVectorBool;
typedef QVector<float> qVectorFloat;


inline QVariant convertToVariant(float v) { return QVariant(v); }
inline QVariant convertToVariant(int v) { return QVariant(v); }
inline QVariant convertToVariant(bool v) { return QVariant(v); }
inline QVariant convertToVariant(quint16 v) { return QVariant(v); }
inline QVariant convertToVariant(quint32 v) { return QVariant(v); }
inline QVariant convertToVariant(quint64 v) { return QVariant(v); }
inline QVariant convertToVariant(const QString& v) { return QVariant(v); }
inline QVariant convertToVariant(const QByteArray& v) { return QVariant(v); }
inline QVariant convertToVariant(const QVariant& v) { return v; }
inline QVariant convertToVariant(const glmVec3& v) { return vec3toVariant(v); }
inline QVariant convertToVariant(const xColor& v) { return xColorToVariant(v); }
inline QVariant convertToVariant(const glmQuat& v) { return quatToVariant(v); }
inline QVariant convertToVariant(const qVectorVec3& v) { return qVectorVec3ToVariant(v); }
inline QVariant convertToVariant(const qVectorQuat& v) { return qVectorQuatToVariant(v); }
inline QVariant convertToVariant(const qVectorBool& v) { return qVectorBoolToVariant(v); }
inline QVariant convertToVariant(const qVectorFloat& v) { return qVectorFloatToVariant(v); }
inline QVariant convertToVariant(const AACube& v) { return aaCubeToVariant(v); }
inline QVariant convertToVariant(const QUuid& v) { return QVariant(v); }


#define COPY_GROUP_PROPERTY_TO_VARIANT(X, G, g, P, p) \
    if ((desiredProperties.isEmpty() || desiredProperties.getHasProperty(X)) && \
            (!skipDefaults || defaultEntityProperties.get##G().get##P() != get##P())) { \
        QVariantMap groupProperties; \
        if (properties[#g].isValid()) { \
            groupProperties = properties[#g].toMap(); \
        } \
        groupProperties.insert(#p, convertToVariant(get##P())); \
        properties.insert(#g, groupProperties); \
    }

#define COPY_GROUP_PROPERTY_TO_VARIANT_GETTER(X, G, g, P, p, M) \
    if ((desiredProperties.isEmpty() || desiredProperties.getHasProperty(X)) && \
            (!skipDefaults || defaultEntityProperties.get##G().get##P() != get##P())) { \
        QVariantMap groupProperties; \
        if (properties[#g].isValid()) { \
            groupProperties = properties[#g].toMap(); \
        } \
        groupProperties.insert(#p, convertToVariant(M())); \
        properties.insert(#g, groupProperties); \
    }

#define COPY_PROPERTY_TO_VARIANT(p, P) \
    if ((_desiredProperties.isEmpty() || _desiredProperties.getHasProperty(p)) && \
            (!skipDefaults || defaultEntityProperties._##P != _##P)) { \
        properties.insert(#P, convertToVariant(_##P)); \
    }

#define COPY_PROPERTY_TO_VARIANT_GETTER_NO_SKIP(P, G) \
    properties.insert(#P, G);

#define COPY_PROPERTY_TO_VARIANT_GETTER(p, P, G) \
    if ((_desiredProperties.isEmpty() || _desiredProperties.getHasProperty(p)) && \
            (!skipDefaults || defaultEntityProperties._##P != _##P)) { \
        properties.insert(#P, convertToVariant(G)); \
    }

// same as COPY_PROPERTY_TO_QSCRIPTVALUE_GETTER but uses #X instead of #P in the setProperty() step
#define COPY_PROXY_PROPERTY_TO_VARIANT_GETTER(p, P, X, G) \
    if ((_desiredProperties.isEmpty() || _desiredProperties.getHasProperty(p)) && \
            (!skipDefaults || defaultEntityProperties._##P != _##P)) { \
        properties.insert(#X, convertToVariant(G)); \
    }

#define COPY_PROPERTY_TO_VARIANT_GETTER_ALWAYS(P, G) \
    if (!skipDefaults || defaultEntityProperties._##P != _##P) { \
        properties.insert(#P, convertToVariant(G)); \
    }


// TODO: make sure all conversions work
inline float float_convertFromVariant(const QVariant& v, bool& isValid) { return v.toFloat(&isValid); }
inline quint64 quint64_convertFromVariant(const QVariant& v, bool& isValid) { return v.toULongLong(&isValid); }
// Use QString::toUInt() so that isValid is set to false if the number is outside the quint32 range.
// FIXME: do something less dumb, maybe a size check
inline quint32 quint32_convertFromVariant(const QVariant& v, bool& isValid) { return v.toString().toUInt(&isValid); }
inline quint16 quint16_convertFromVariant(const QVariant& v, bool& isValid) { return v.toInt(&isValid); }
inline uint16_t uint16_t_convertFromVariant(const QVariant& v, bool& isValid) { return v.toInt(&isValid); }
inline int int_convertFromVariant(const QVariant& v, bool& isValid) { return v.toInt(&isValid); }
inline bool bool_convertFromVariant(const QVariant& v, bool& isValid) { isValid = true; return v.toBool(); }
inline uint8_t uint8_t_convertFromVariant(const QVariant& v, bool& isValid)
    { isValid = true; return (uint8_t)(0xff & v.toInt(&isValid)); }
inline QString QString_convertFromVariant(const QVariant& v, bool& isValid) { isValid = true; return v.toString().trimmed(); }
inline QUuid QUuid_convertFromVariant(const QVariant& v, bool& isValid) { isValid = true; return v.toUuid(); }
inline EntityItemID EntityItemID_convertFromVariant(const QVariant& v, bool& isValid) { isValid = true; return v.toUuid(); }
inline QDateTime QDateTime_convertFromVariant(const QVariant& v, bool& isValid) { isValid = true; return v.toDateTime(); }
inline QByteArray QByteArray_convertFromVariant(const QVariant& v, bool& isValid) { isValid = true; return v.toByteArray(); }
inline glmVec3 glmVec3_convertFromVariant(const QVariant& v, bool& isValid) { return vec3FromVariant(v, isValid); }
inline glmQuat glmQuat_convertFromVariant(const QVariant& v, bool& isValid) { return quatFromVariant(v, isValid); }
inline AACube AACube_convertFromVariant(const QVariant& v, bool& isValid) { isValid = true; return aaCubeFromVariant(v); }
inline xColor xColor_convertFromVariant(const QVariant& v, bool& isValid) { return xColorFromVariant(v, isValid); }
inline qVectorFloat qVectorFloat_convertFromVariant(const QVariant& v, bool& isValid)
    { isValid = true; return qVectorFloatFromVariant(v); }
inline qVectorVec3 qVectorVec3_convertFromVariant(const QVariant& v, bool& isValid)
    { isValid = true; return qVectorVec3FromVariant(v); }
inline qVectorQuat qVectorQuat_convertFromVariant(const QVariant& v, bool& isValid)
    { isValid = true; return qVectorQuatFromVariant(v); }
inline qVectorBool qVectorBool_convertFromVariant(const QVariant& v, bool& isValid)
    { isValid = true; return qVectorBoolFromVariant(v); }


#define COPY_PROPERTY_FROM_VARIANT(P, T, S) \
    { \
        QVariant V = map[#P]; \
        if (V.isValid()) { \
            bool isValid = false; \
            T newValue = T##_convertFromVariant(V, isValid); \
            if (isValid && (_defaultSettings || newValue != _##P)) { \
                S(newValue); \
            } \
        } \
    }

#define COPY_PROPERTY_FROM_VARIANT_GETTER(P, T, S, G) \
    { \
        QVariant V = map[#P]; \
        if (V.isValid()) { \
            bool isValid = false; \
            T newValue = T##_convertFromVariant(V, isValid); \
            if (isValid && (_defaultSettings || newValue != G())) { \
                S(newValue); \
            } \
        } \
    }

#define COPY_PROPERTY_FROM_VARIANT_NOCHECK(P, T, S) \
    { \
        QVariant V = map[#P]; \
        if (V.isValid()) { \
            bool isValid = false; \
            T newValue = T##_convertFromVariant(V, isValid); \
            if (isValid && (_defaultSettings)) { \
                S(newValue); \
            } \
        } \
    }

#define COPY_GROUP_PROPERTY_FROM_VARIANT(G, P, T, S) \
    { \
        QVariant g = map[#G]; \
        if (g.isValid()) { \
            QVariant V = g.toMap()[#P]; \
            if (V.isValid()) { \
                bool isValid = false; \
                T newValue = T##_convertFromVariant(V, isValid); \
                if (isValid && (_defaultSettings || newValue != _##P)) { \
                    S(newValue); \
                } \
            } \
        } \
    }

#define COPY_PROPERTY_FROM_VARIANT_ENUM(P, S) \
    { \
        QVariant p = map[#P]; \
        if (p.isValid()) { \
            QString newValue = p.toString(); \
            if (_defaultSettings || newValue != get##S##AsString()) { \
                set##S##FromString(newValue); \
            } \
        } \
    }


#define DEFINE_PROPERTY_GROUP(N, n, T)           \
    public:                                      \
        const T& get##N() const { return _##n; } \
        T& get##N() { return _##n; }             \
    private:                                     \
        T _##n;                                  \
        static T _static##N; 

#define ADD_PROPERTY_TO_MAP(P, N, n, T) \
        _propertyStringsToEnums[#n] = P;

#define ADD_GROUP_PROPERTY_TO_MAP(P, G, g, N, n) \
        _propertyStringsToEnums[#g "." #n] = P;

#define DEFINE_CORE(N, n, T, V) \
    public: \
        bool n##Changed() const { return _##n##Changed; } \
        void set##N##Changed(bool value) { _##n##Changed = value; } \
    private: \
        T _##n = V; \
        bool _##n##Changed { false };

#define DEFINE_PROPERTY(P, N, n, T, V)        \
    public: \
        T get##N() const { return _##n; } \
        void set##N(T value) { _##n = value; _##n##Changed = true; } \
    DEFINE_CORE(N, n, T, V)

#define DEFINE_PROPERTY_REF(P, N, n, T, V)        \
    public: \
        const T& get##N() const { return _##n; } \
        void set##N(const T& value) { _##n = value; _##n##Changed = true; } \
    DEFINE_CORE(N, n, T, V)

#define DEFINE_PROPERTY_REF_WITH_SETTER(P, N, n, T, V)        \
    public: \
        const T& get##N() const { return _##n; } \
        void set##N(const T& value); \
    DEFINE_CORE(N, n, T, V)

#define DEFINE_PROPERTY_REF_WITH_SETTER_AND_GETTER(P, N, n, T, V)        \
    public: \
        T get##N() const; \
        void set##N(const T& value); \
    DEFINE_CORE(N, n, T, V)

#define DEFINE_PROPERTY_REF_ENUM(P, N, n, T, V) \
    public: \
        const T& get##N() const { return _##n; } \
        void set##N(const T& value) { _##n = value; _##n##Changed = true; } \
        QString get##N##AsString() const; \
        void set##N##FromString(const QString& name); \
    DEFINE_CORE(N, n, T, V)

#define DEBUG_PROPERTY(D, P, N, n, x)                \
    D << "  " << #n << ":" << P.get##N() << x << "[changed:" << P.n##Changed() << "]\n";

#define DEBUG_PROPERTY_IF_CHANGED(D, P, N, n, x)                \
    if (P.n##Changed()) {                                       \
        D << "  " << #n << ":" << P.get##N() << x << "\n";      \
    }

#endif // hifi_EntityItemPropertiesMacros_h
