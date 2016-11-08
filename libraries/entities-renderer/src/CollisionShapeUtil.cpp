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

#include "CollisionShapeUtil.h"

#include <assert.h>
#include <glm/gtx/transform.hpp>

const uint32_t TRIANGLE_STRIDE = 3;
const uint32_t QUAD_STRIDE = 4;


void CollisionShapeUtil::computeShapeInfoFromGeometryResource(
        GeometryResource::Pointer geometryResource,
		ModelPointer visualModel,
        const glm::vec3& postOffset,
        ShapeInfo& shapeInfo) {
    assert(shapeInfo.getType()  == SHAPE_TYPE_COMPOUND);

    const FBXGeometry& collisionGeometry = geometryResource->getFBXGeometry();

    ShapeInfo::PointCollection& pointCollection = shapeInfo.getPointCollection();
    pointCollection.clear();

    // the way OBJ files get read, each section under a "g" line is its own meshPart.  We only expect
    // to find one actual "mesh" (with one or more meshParts in it), but we loop over the meshes, just in case.
    uint32_t i = 0;
    foreach (const FBXMesh& mesh, collisionGeometry.meshes) {
        // each meshPart is a convex hull
        foreach (const FBXMeshPart &meshPart, mesh.parts) {
            pointCollection.push_back(QVector<glm::vec3>());
            ShapeInfo::PointList& pointsInPart = pointCollection[i];

            // run through all the triangles and (uniquely) add each point to the hull
            uint32_t numIndices = (uint32_t)meshPart.triangleIndices.size();
            // TODO: assert rather than workaround after we start sanitizing FBXMesh higher up
            //assert(numIndices % TRIANGLE_STRIDE == 0);
            numIndices -= numIndices % TRIANGLE_STRIDE; // WORKAROUND lack of sanity checking in FBXReader

            for (uint32_t j = 0; j < numIndices; j += TRIANGLE_STRIDE) {
                glm::vec3 p0 = mesh.vertices[meshPart.triangleIndices[j]];
                glm::vec3 p1 = mesh.vertices[meshPart.triangleIndices[j + 1]];
                glm::vec3 p2 = mesh.vertices[meshPart.triangleIndices[j + 2]];
                if (!pointsInPart.contains(p0)) {
                    pointsInPart << p0;
                }
                if (!pointsInPart.contains(p1)) {
                    pointsInPart << p1;
                }
                if (!pointsInPart.contains(p2)) {
                    pointsInPart << p2;
                }
            }

            // run through all the quads and (uniquely) add each point to the hull
            numIndices = (uint32_t)meshPart.quadIndices.size();
            // TODO: assert rather than workaround after we start sanitizing FBXMesh higher up
            //assert(numIndices % QUAD_STRIDE == 0);
            numIndices -= numIndices % QUAD_STRIDE; // WORKAROUND lack of sanity checking in FBXReader

            for (uint32_t j = 0; j < numIndices; j += QUAD_STRIDE) {
                glm::vec3 p0 = mesh.vertices[meshPart.quadIndices[j]];
                glm::vec3 p1 = mesh.vertices[meshPart.quadIndices[j + 1]];
                glm::vec3 p2 = mesh.vertices[meshPart.quadIndices[j + 2]];
                glm::vec3 p3 = mesh.vertices[meshPart.quadIndices[j + 3]];
                if (!pointsInPart.contains(p0)) {
                    pointsInPart << p0;
                }
                if (!pointsInPart.contains(p1)) {
                    pointsInPart << p1;
                }
                if (!pointsInPart.contains(p2)) {
                    pointsInPart << p2;
                }
                if (!pointsInPart.contains(p3)) {
                    pointsInPart << p3;
                }
            }

            if (pointsInPart.size() == 0) {
                pointCollection.pop_back();
                continue;
            }
            ++i;
        }
    }

    // We expect that the collision model will have the same units and will be displaced
    // from its origin in the same way the visualModel is.  The visualModel has
    // been centered and probably scaled.  We take the scaling and offset which were applied
    // to the visualModel and apply them to the collision model (without regard for the
    // collision model's extents).
	assert((bool)visualModel);

    // multiply each point by scale before handing the point-set off to the physics engine.
    // also determine the extents of the collision model.
    glm::vec3 scaleToFit = 2.0f * shapeInfo.getHalfExtents() / visualModel->getFBXGeometry().getUnscaledMeshExtents().size();
	glm::vec3 preOffset = visualModel->getOffset();
    for (int32_t i = 0; i < pointCollection.size(); i++) {
        for (int32_t j = 0; j < pointCollection[i].size(); j++) {
            pointCollection[i][j] = scaleToFit * (pointCollection[i][j] + preOffset) + postOffset;
        }
    }
}

void CollisionShapeUtil::computeShapeInfoFromModel(ModelPointer model, const glm::vec3& postOffset, ShapeInfo& shapeInfo) {
    int32_t type = shapeInfo.getType();
    assert(type >= (int32_t)SHAPE_TYPE_SIMPLE_HULL && type <= (int32_t)SHAPE_TYPE_STATIC_MESH);

    // compute meshPart local transforms
    QVector<glm::mat4> localTransforms;
    const FBXGeometry& fbxGeometry = model->getFBXGeometry();
    int32_t numFbxMeshes = fbxGeometry.meshes.size();
    int32_t totalNumVertices = 0;
    glm::mat4 offsetTranslation = glm::translate(postOffset);
    for (int32_t i = 0; i < numFbxMeshes; i++) {
        const FBXMesh& mesh = fbxGeometry.meshes.at(i);
        if (mesh.clusters.size() > 0) {
            const FBXCluster& cluster = mesh.clusters.at(0);
            auto jointMatrix = model->getRig()->getJointTransform(cluster.jointIndex);
            localTransforms.push_back(offsetTranslation * jointMatrix * cluster.inverseBindMatrix);
        } else {
            glm::mat4 identity;
            localTransforms.push_back(offsetTranslation);
        }
        totalNumVertices += mesh.vertices.size();

        const int32_t MAX_VERTICES_PER_STATIC_MESH = 1e6;
        if (totalNumVertices > MAX_VERTICES_PER_STATIC_MESH) {
            qWarning() << "model" << model->getURL() << "has too many vertices" << totalNumVertices << "and will collide as a box.";
            shapeInfo.setParams(SHAPE_TYPE_BOX, shapeInfo.getHalfExtents());
            return;
        }
    }

    auto& meshes = model->getGeometry()->getMeshes();
    int32_t numMeshes = (int32_t)(meshes.size());

    ShapeInfo::PointCollection& pointCollection = shapeInfo.getPointCollection();
    pointCollection.clear();
    if (type == SHAPE_TYPE_SIMPLE_COMPOUND) {
        pointCollection.resize(numMeshes);
    } else {
        pointCollection.resize(1);
    }

    ShapeInfo::TriangleIndices& triangleIndices = shapeInfo.getTriangleIndices();
    triangleIndices.clear();

    Extents extents;
    int32_t meshCount = 0;
    int32_t pointListIndex = 0;
    for (auto& mesh : meshes) {
        if (!mesh) {
            continue;
        }
        const gpu::BufferView& vertices = mesh->getVertexBuffer();
        const gpu::BufferView& indices = mesh->getIndexBuffer();
        const gpu::BufferView& parts = mesh->getPartBuffer();

        ShapeInfo::PointList& points = pointCollection[pointListIndex];

        // reserve room
        int32_t sizeToReserve = (int32_t)(vertices.getNumElements());
        if (type == SHAPE_TYPE_SIMPLE_COMPOUND) {
            // a list of points for each mesh
            pointListIndex++;
        } else {
            // only one list of points
            sizeToReserve += (int32_t)((gpu::Size)points.size());
        }
        points.reserve(sizeToReserve);

        // copy points
        uint32_t meshIndexOffset = (uint32_t)points.size();
        const glm::mat4& localTransform = localTransforms[meshCount];
        gpu::BufferView::Iterator<const glm::vec3> vertexItr = vertices.cbegin<const glm::vec3>();
        while (vertexItr != vertices.cend<const glm::vec3>()) {
            glm::vec3 point = extractTranslation(localTransform * glm::translate(*vertexItr));
            points.push_back(point);
            extents.addPoint(point);
            ++vertexItr;
        }

        if (type == SHAPE_TYPE_STATIC_MESH) {
            // copy into triangleIndices
            triangleIndices.reserve((int32_t)((gpu::Size)(triangleIndices.size()) + indices.getNumElements()));
            gpu::BufferView::Iterator<const model::Mesh::Part> partItr = parts.cbegin<const model::Mesh::Part>();
            while (partItr != parts.cend<const model::Mesh::Part>()) {
                auto numIndices = partItr->_numIndices;
                if (partItr->_topology == model::Mesh::TRIANGLES) {
                // TODO: assert rather than workaround after we start sanitizing FBXMesh higher up
                    //assert(numIndices % TRIANGLE_STRIDE == 0);
                    numIndices -= numIndices % TRIANGLE_STRIDE; // WORKAROUND lack of sanity checking in FBXReader

                    auto indexItr = indices.cbegin<const gpu::BufferView::Index>() + partItr->_startIndex;
                    auto indexEnd = indexItr + numIndices;
                    while (indexItr != indexEnd) {
                        triangleIndices.push_back(*indexItr + meshIndexOffset);
                        ++indexItr;
                    }
                } else if (partItr->_topology == model::Mesh::TRIANGLE_STRIP) {
                    // TODO: resurrect assert after we start sanitizing FBXMesh higher up
                    //assert(numIndices > 2);

                    uint32_t approxNumIndices = TRIANGLE_STRIDE * numIndices;
                    if (approxNumIndices > (uint32_t)(triangleIndices.capacity() - triangleIndices.size())) {
                        // we underestimated the final size of triangleIndices so we pre-emptively expand it
                        triangleIndices.reserve(triangleIndices.size() + approxNumIndices);
                    }

                    auto indexItr = indices.cbegin<const gpu::BufferView::Index>() + partItr->_startIndex;
                    auto indexEnd = indexItr + (numIndices - 2);

                    // first triangle uses the first three indices
                    triangleIndices.push_back(*(indexItr++) + meshIndexOffset);
                    triangleIndices.push_back(*(indexItr++) + meshIndexOffset);
                    triangleIndices.push_back(*(indexItr++) + meshIndexOffset);

                    // the rest use previous and next index
                    uint32_t triangleCount = 1;
                    while (indexItr != indexEnd) {
                        if ((*indexItr) != model::Mesh::PRIMITIVE_RESTART_INDEX) {
                            if (triangleCount % 2 == 0) {
                                // even triangles use first two indices in order
                                triangleIndices.push_back(*(indexItr - 2) + meshIndexOffset);
                                triangleIndices.push_back(*(indexItr - 1) + meshIndexOffset);
                            } else {
                                // odd triangles swap order of first two indices
                                triangleIndices.push_back(*(indexItr - 1) + meshIndexOffset);
                                triangleIndices.push_back(*(indexItr - 2) + meshIndexOffset);
                            }
                            triangleIndices.push_back(*indexItr + meshIndexOffset);
                            ++triangleCount;
                        }
                        ++indexItr;
                    }
                }
                ++partItr;
            }
        } else if (type == SHAPE_TYPE_SIMPLE_COMPOUND) {
            // for each mesh copy unique part indices, separated by special bogus (flag) index values
            gpu::BufferView::Iterator<const model::Mesh::Part> partItr = parts.cbegin<const model::Mesh::Part>();
            while (partItr != parts.cend<const model::Mesh::Part>()) {
                // collect unique list of indices for this part
                std::set<int32_t> uniqueIndices;
                auto numIndices = partItr->_numIndices;
                if (partItr->_topology == model::Mesh::TRIANGLES) {
                    // TODO: assert rather than workaround after we start sanitizing FBXMesh higher up
                    //assert(numIndices% TRIANGLE_STRIDE == 0);
                    numIndices -= numIndices % TRIANGLE_STRIDE; // WORKAROUND lack of sanity checking in FBXReader

                    auto indexItr = indices.cbegin<const gpu::BufferView::Index>() + partItr->_startIndex;
                    auto indexEnd = indexItr + numIndices;
                    while (indexItr != indexEnd) {
                        uniqueIndices.insert(*indexItr);
                        ++indexItr;
                    }
                } else if (partItr->_topology == model::Mesh::TRIANGLE_STRIP) {
                    // TODO: resurrect assert after we start sanitizing FBXMesh higher up
                    //assert(numIndices > TRIANGLE_STRIDE - 1);

                    auto indexItr = indices.cbegin<const gpu::BufferView::Index>() + partItr->_startIndex;
                    auto indexEnd = indexItr + (numIndices - 2);

                    // first triangle uses the first three indices
                    uniqueIndices.insert(*(indexItr++));
                    uniqueIndices.insert(*(indexItr++));
                    uniqueIndices.insert(*(indexItr++));

                    // the rest use previous and next index
                    uint32_t triangleCount = 1;
                    while (indexItr != indexEnd) {
                        if ((*indexItr) != model::Mesh::PRIMITIVE_RESTART_INDEX) {
                            if (triangleCount % 2 == 0) {
                                // EVEN triangles use first two indices in order
                                uniqueIndices.insert(*(indexItr - 2));
                                uniqueIndices.insert(*(indexItr - 1));
                            } else {
                                // ODD triangles swap order of first two indices
                                uniqueIndices.insert(*(indexItr - 1));
                                uniqueIndices.insert(*(indexItr - 2));
                            }
                            uniqueIndices.insert(*indexItr);
                            ++triangleCount;
                        }
                        ++indexItr;
                    }
                }

                // store uniqueIndices in triangleIndices
                triangleIndices.reserve(triangleIndices.size() + (int32_t)uniqueIndices.size());
                for (auto index : uniqueIndices) {
                    triangleIndices.push_back(index);
                }
                // flag end of part
                triangleIndices.push_back(END_OF_MESH_PART);

                ++partItr;
            }
            // flag end of mesh
            triangleIndices.push_back(END_OF_MESH);
        }
        ++meshCount;
    }

    // scale and shift
    glm::vec3 halfExtents = 0.5f * extents.size();
    glm::vec3 scaleToFit = shapeInfo.getHalfExtents() / halfExtents;
    for (int32_t i = 0; i < 3; ++i) {
        if (halfExtents[i] < 1.0e-6f) {
            scaleToFit[i] = 1.0f;
        }
    }
    for (auto points : pointCollection) {
        for (int32_t i = 0; i < points.size(); ++i) {
            points[i] = (points[i] * scaleToFit);
        }
    }
}

