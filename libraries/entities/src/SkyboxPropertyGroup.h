//
//  SkyboxPropertyGroup.h
//  libraries/entities/src
//
//  Created by Brad Hefta-Gaub on 12/4/13.
//  Copyright 2013 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_SkyboxPropertyGroup_h
#define hifi_SkyboxPropertyGroup_h

#include <stdint.h>

#include <glm/glm.hpp>

#include <QtScript/QScriptEngine>

#include "PropertyGroup.h"
#include "EntityItemPropertiesMacros.h"

class EntityItemProperties;
class EncodeBitstreamParams;
class OctreePacketData;
class EntityTreeElementExtraEncodeData;
class ReadBitstreamToTreeParams;

class SkyboxPropertyGroup : public PropertyGroup {
public:
    // EntityItemProperty related helpers
    virtual void copyToVariant(const EntityPropertyFlags& desiredProperties, QVariantMap& properties, bool skipDefaults,
                               const EntityItemProperties& defaultEntityProperties) const;
    virtual void copyFromVariant(const QVariantMap& map, bool& _defaultSettings);
    virtual void debugDump() const;
    virtual void listChangedProperties(QList<QString>& out);

    virtual bool appendToEditPacket(OctreePacketData* packetData,
                                    EntityPropertyFlags& requestedProperties,
                                    EntityPropertyFlags& propertyFlags,
                                    EntityPropertyFlags& propertiesDidntFit,
                                    int& propertyCount, 
                                    OctreeElement::AppendState& appendState) const;

    virtual bool decodeFromEditPacket(EntityPropertyFlags& propertyFlags, const unsigned char*& dataAt , int& processedBytes);
    virtual void markAllChanged();
    virtual EntityPropertyFlags getChangedProperties() const;

    // EntityItem related helpers
    // methods for getting/setting all properties of an entity
    virtual void getProperties(EntityItemProperties& propertiesOut) const;
    
    /// returns true if something changed
    virtual bool setProperties(const EntityItemProperties& properties);

    virtual EntityPropertyFlags getEntityProperties(EncodeBitstreamParams& params) const;
        
    virtual void appendSubclassData(OctreePacketData* packetData, EncodeBitstreamParams& params, 
                                    EntityTreeElementExtraEncodeData* entityTreeElementExtraEncodeData,
                                    EntityPropertyFlags& requestedProperties,
                                    EntityPropertyFlags& propertyFlags,
                                    EntityPropertyFlags& propertiesDidntFit,
                                    int& propertyCount, 
                                    OctreeElement::AppendState& appendState) const;

    virtual int readEntitySubclassDataFromBuffer(const unsigned char* data, int bytesLeftToRead, 
                                                ReadBitstreamToTreeParams& args,
                                                EntityPropertyFlags& propertyFlags, bool overwriteLocalData,
                                                bool& somethingChanged);
                                                
    glm::vec3 getColorVec3() const {
        const quint8 MAX_COLOR = 255;
        glm::vec3 color = { (float)_color.red / (float)MAX_COLOR,
                            (float)_color.green / (float)MAX_COLOR,
                            (float)_color.blue / (float)MAX_COLOR };
        return color;
    }

    static const xColor DEFAULT_COLOR;
    DEFINE_PROPERTY_REF(PROP_SKYBOX_COLOR, Color, color, xColor, DEFAULT_COLOR);
    DEFINE_PROPERTY_REF(PROP_SKYBOX_URL, URL, url, QString, "");
};

#endif // hifi_SkyboxPropertyGroup_h
