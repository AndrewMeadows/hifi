//
//  CollisionRenderMeshCache.cpp
//  libraries/physcis/src
//
//  Created by Andrew Meadows 2016.07.13
//  Copyright 2016 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "CollisionRenderMeshCache.h"

#include <cassert>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>

#include <ShapeInfo.h> // for MAX_HULL_POINTS

const int32_t MAX_HULL_INDICES = 6 * MAX_HULL_POINTS;
float tempVertexBuffer[3 * MAX_HULL_POINTS];
model::Index tempIndexBuffer[MAX_HULL_INDICES];

//void copyShapeToMesh(const btTransform& transform, const btConvexShape* shape, model::MeshPointer mesh) {
void copyShapeToMesh(const btTransform& transform, const btConvexShape* shape,
        gpu::BufferView& vertices, gpu::BufferView& indices, gpu::BufferView& parts) {
    assert(shape);

    btShapeHull hull(shape);
    const btScalar MARGIN = 0.0f;
    hull.buildHull(MARGIN);

    int32_t numHullIndices = hull.numIndices();
    assert(numHullIndices <= MAX_HULL_INDICES);

    int32_t numHullVertices = hull.numVertices();
    assert(numHullVertices <= MAX_HULL_POINTS);

    { // new part
        model::Mesh::Part part;
        part._startIndex = indices.getNumElements();
        part._numIndices = (model::Index)numHullIndices;
        // FIXME: the render code cannot handle the case where part._baseVertex != 0
        //part._baseVertex = vertices.getNumElements(); // DOES NOT WORK
        part._baseVertex = 0;

        gpu::BufferView::Size numBytes = sizeof(model::Mesh::Part);
        const gpu::Byte* data = reinterpret_cast<const gpu::Byte*>(&part);
        parts._buffer->append(numBytes, data);
        parts._size = parts._buffer->getSize();
    }

    model::Index indexOffset = vertices.getNumElements();
    { // new vertices
        const btVector3* hullVertices = hull.getVertexPointer();
        assert(numHullVertices <= MAX_HULL_POINTS);
        for (int32_t i = 0; i < numHullVertices; ++i) {
            btVector3 transformedPoint = transform * hullVertices[i];
            memcpy(tempVertexBuffer + 3 * i, transformedPoint.m_floats, 3 * sizeof(float));
        }

        gpu::BufferView::Size numBytes = sizeof(float) * (3 * numHullVertices);
        const gpu::Byte* data = reinterpret_cast<const gpu::Byte*>(tempVertexBuffer);
        vertices._buffer->append(numBytes, data);
        vertices._size = vertices._buffer->getSize();
    }

    { // new indices
        const uint32_t* hullIndices = hull.getIndexPointer();
        // FIXME: the render code cannot handle the case where part._baseVertex != 0
        // so we must add an offset to each index
        for (int32_t i = 0; i < numHullIndices; ++i) {
            tempIndexBuffer[i] = hullIndices[i] + indexOffset;
        }
        const gpu::Byte* data = reinterpret_cast<const gpu::Byte*>(tempIndexBuffer);
        gpu::BufferView::Size numBytes = (gpu::BufferView::Size)(sizeof(model::Index) * numHullIndices);
        indices._buffer->append(numBytes, data);
        indices._size = indices._buffer->getSize();
    }

    gpu::BufferView::Iterator<const model::Mesh::Part> partItr = parts.cbegin<const model::Mesh::Part>();
    gpu::Size numParts = parts.getNumElements();
}

model::MeshPointer createMeshFromShape(const void* pointer) {
    model::MeshPointer mesh;
    if (!pointer) {
        return mesh;
    }

    // pointer must be a const btCollisionShape* (cast to void*), but it only
    // needs to be valid here when its render mesh is created, after this call
    // the cache doesn't care what happens to the shape behind the pointer
    const btCollisionShape* shape = static_cast<const btCollisionShape*>(pointer);

    int32_t shapeType = shape->getShapeType();
    if (shapeType == (int32_t)COMPOUND_SHAPE_PROXYTYPE || shape->isConvex()) {
        // create the mesh and allocate buffers for it
        mesh = std::make_shared<model::Mesh>();
        gpu::BufferView vertices(new gpu::Buffer(), mesh->getVertexBuffer()._element);
        gpu::BufferView indices(new gpu::Buffer(), mesh->getIndexBuffer()._element);
        gpu::BufferView parts(new gpu::Buffer(), mesh->getPartBuffer()._element);

        if (shapeType == (int32_t)COMPOUND_SHAPE_PROXYTYPE) {
            const btCompoundShape* compoundShape = static_cast<const btCompoundShape*>(shape);
            int32_t numSubShapes = compoundShape->getNumChildShapes();
            for (int32_t i = 0; i < numSubShapes; ++i) {
                const btCollisionShape* childShape = compoundShape->getChildShape(i);
                if (childShape->isConvex()) {
                    const btConvexShape* convexShape = static_cast<const btConvexShape*>(childShape);
                    copyShapeToMesh(compoundShape->getChildTransform(i), convexShape, vertices, indices, parts);
                }
            }
        } else {
            // shape is convex
            const btConvexShape* convexShape = static_cast<const btConvexShape*>(shape);
            btTransform transform;
            transform.setIdentity();
            copyShapeToMesh(transform, convexShape, vertices, indices, parts);
        }
        mesh->setVertexBuffer(vertices);
        mesh->setIndexBuffer(indices);
        mesh->setPartBuffer(parts);
    }
    return mesh;
}

CollisionRenderMeshCache::CollisionRenderMeshCache() {
}

CollisionRenderMeshCache::~CollisionRenderMeshCache() {
    _meshMap.clear();
    _pendingGarbage.clear();
}

model::MeshPointer CollisionRenderMeshCache::getMesh(CollisionRenderMeshCache::Key key) {
    model::MeshPointer mesh;
    if (key) {
        CollisionMeshMap::const_iterator itr = _meshMap.find(key);
        if (itr == _meshMap.end()) {
            // make mesh and add it to map
            mesh = createMeshFromShape(key);
            if (mesh) {
                _meshMap.insert(std::make_pair(key, mesh));
            }
        } else {
            mesh = itr->second;
        }
    }
    const uint32_t MAX_NUM_PENDING_GARBAGE = 20;
    if (_pendingGarbage.size() > MAX_NUM_PENDING_GARBAGE) {
        collectGarbage();
    }
    return mesh;
}

bool CollisionRenderMeshCache::releaseMesh(CollisionRenderMeshCache::Key key) {
    if (!key) {
        return false;
    }
    CollisionMeshMap::const_iterator itr = _meshMap.find(key);
    if (itr != _meshMap.end()) {
        // we hold at least one reference, and the outer scope also holds at least one
        // so we assert that the reference count is not 1
        assert((*itr).second.use_count() != 1);

        _pendingGarbage.push_back(key);
        return true;
    }
    return false;
}

void CollisionRenderMeshCache::collectGarbage() {
    uint32_t numShapes = _pendingGarbage.size();
    for (uint32_t i = 0; i < numShapes; ++i) {
        CollisionRenderMeshCache::Key key = _pendingGarbage[i];
        CollisionMeshMap::const_iterator itr = _meshMap.find(key);
        if (itr != _meshMap.end()) {
            if ((*itr).second.use_count() == 1) {
                // we hold the only reference
                _meshMap.erase(itr);
            }
        }
    }
    _pendingGarbage.clear();
}

