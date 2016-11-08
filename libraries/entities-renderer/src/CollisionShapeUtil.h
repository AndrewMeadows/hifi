//
//  CollisionShapeUtils.h
//  entities-renderer/src/entities
//
//  Created by Andrew Meadows 2016.11.08
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_CollisionShapeUtil_h
#define hifi_CollisionShapeUtil_h

#include <FBXReader.h>
#include <Model.h>
#include <ShapeInfo.h>

namespace CollisionShapeUtil {

	void computeShapeInfoFromGeometryResource(
        	GeometryResource::Pointer geometryResource,
			ModelPointer visualModel,
        	const glm::vec3& postOffset,
        	ShapeInfo& shapeInfo);

    void computeShapeInfoFromModel(ModelPointer model, const glm::vec3& postOffset, ShapeInfo& shapeInfo);

};

#endif // hifi_CollisionShapeUtil_h
