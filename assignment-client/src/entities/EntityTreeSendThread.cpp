//
//  EntityTreeSendThread.cpp
//  assignment-client/src/entities
//
//  Created by Stephen Birarda on 2/15/17.
//  Copyright 2017 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "EntityTreeSendThread.h"

#include <EntityNodeData.h>
#include <EntityTypes.h>
#include <OctreeUtils.h>

#include "EntityServer.h"

//#define SEND_SORTED_ENTITIES

void EntityTreeSendThread::preDistributionProcessing() {
    auto node = _node.toStrongRef();
    auto nodeData = static_cast<EntityNodeData*>(node->getLinkedData());

    if (nodeData) {
        auto jsonQuery = nodeData->getJSONParameters();

        // check if we have a JSON query with flags
        auto flags = jsonQuery[EntityJSONQueryProperties::FLAGS_PROPERTY].toObject();
        if (!flags.isEmpty()) {
            // check the flags object for specific flags that require special pre-processing

            bool includeAncestors = flags[EntityJSONQueryProperties::INCLUDE_ANCESTORS_PROPERTY].toBool();
            bool includeDescendants = flags[EntityJSONQueryProperties::INCLUDE_DESCENDANTS_PROPERTY].toBool();

            if (includeAncestors || includeDescendants) {
                // we need to either include the ancestors, descendants, or both for entities matching the filter
                // included in the JSON query

                // first reset our flagged extra entities so we start with an empty set
                nodeData->resetFlaggedExtraEntities();

                auto entityTree = std::static_pointer_cast<EntityTree>(_myServer->getOctree());

                bool requiresFullScene = false;

                // enumerate the set of entity IDs we know currently match the filter
                foreach(const QUuid& entityID, nodeData->getSentFilteredEntities()) {
                    if (includeAncestors) {
                        // we need to include ancestors - recurse up to reach them all and add their IDs
                        // to the set of extra entities to include for this node
                        entityTree->withReadLock([&]{
                            auto filteredEntity = entityTree->findEntityByID(entityID);
                            if (filteredEntity) {
                                requiresFullScene |= addAncestorsToExtraFlaggedEntities(entityID, *filteredEntity, *nodeData);
                            }
                        });
                    }

                    if (includeDescendants) {
                        // we need to include descendants - recurse down to reach them all and add their IDs
                        // to the set of extra entities to include for this node
                        entityTree->withReadLock([&]{
                            auto filteredEntity = entityTree->findEntityByID(entityID);
                            if (filteredEntity) {
                                requiresFullScene |= addDescendantsToExtraFlaggedEntities(entityID, *filteredEntity, *nodeData);
                            }
                        });
                    }
                }

                if (requiresFullScene) {
                    // for one or more of the entities matching our filter we found new extra entities to include

                    // because it is possible that one of these entities hasn't changed since our last send
                    // and therefore would not be recursed to, we need to force a full traversal for this pass
                    // of the tree to allow it to grab all of the extra entities we're asking it to include
                    nodeData->setShouldForceFullScene(requiresFullScene);
                }
            }
        }
    }
}

void EntityTreeSendThread::traverseTreeAndSendContents(SharedNodePointer node, OctreeQueryNode* nodeData,
            bool viewFrustumChanged, bool isFullScene) {
    // BEGIN EXPERIMENTAL DIFFERENTIAL TRAVERSAL
    if (nodeData->getUsesFrustum()) {
        // DEBUG HACK: trigger traversal (Repeat) every so often
        const uint64_t TRAVERSE_AGAIN_PERIOD = 4 * USECS_PER_SECOND;
        bool repeatTraversal = usecTimestampNow() > _traversal.getStartOfCompletedTraversal() + TRAVERSE_AGAIN_PERIOD;
        if (viewFrustumChanged || repeatTraversal) {
            ViewFrustum viewFrustum;
            nodeData->copyCurrentViewFrustum(viewFrustum);
            EntityTreeElementPointer root = std::dynamic_pointer_cast<EntityTreeElement>(_myServer->getOctree()->getRoot());
            int32_t lodLevelOffset = nodeData->getBoundaryLevelAdjust() + (viewFrustumChanged ? LOW_RES_MOVING_ADJUST : NO_BOUNDARY_ADJUST);
            startNewTraversal(viewFrustum, root, lodLevelOffset);

            // If the previous traversal didn't finish, we'll need to resort the entities still in _sendQueue after calling traverse
            if (!_sendQueue.empty()) {
                EntityPriorityQueue prevSendQueue;
                _sendQueue.swap(prevSendQueue);
                _entitiesToSend.clear();
                // Re-add elements from previous traversal if they still need to be sent
                while (!prevSendQueue.empty()) {
                    EntityItemPointer entity = prevSendQueue.top().getEntity();
                    prevSendQueue.pop();
                    if (entity) {
                        bool success = false;
                        AACube cube = entity->getQueryAACube(success);
                        if (success) {
                            if (_traversal.getCurrentView().cubeIntersectsKeyhole(cube)) {
                                const float DO_NOT_SEND = -1.0e-6f;
                                float priority = _conicalView.computePriority(cube);
                                if (priority != DO_NOT_SEND) {
                                    float renderAccuracy = calculateRenderAccuracy(_traversal.getCurrentView().getPosition(),
                                                                                   cube,
                                                                                   _traversal.getCurrentRootSizeScale(),
                                                                                   lodLevelOffset);

                                    // Only send entities if they are large enough to see
                                    if (renderAccuracy > 0.0f) {
                                        _sendQueue.push(PrioritizedEntity(entity, priority));
                                        _entitiesToSend.insert(entity);
                                    }
                                }
                            }
                        } else {
                            const float WHEN_IN_DOUBT_PRIORITY = 1.0f;
                            _sendQueue.push(PrioritizedEntity(entity, WHEN_IN_DOUBT_PRIORITY));
                            _entitiesToSend.insert(entity);
                        }
                    }
                }
            }
        }
    }

    if (!_traversal.finished()) {
        uint64_t startTime = usecTimestampNow();

        const uint64_t TIME_BUDGET = 100000; // usec
        _traversal.traverse(TIME_BUDGET);

        uint64_t dt = usecTimestampNow() - startTime;
        std::cout << "adebug  traversal complete " << "  Q.size = " << _sendQueue.size() << "  dt = " << dt << std::endl;  // adebug
    }

#ifndef SEND_SORTED_ENTITIES
    if (!_sendQueue.empty()) {
        // print what needs to be sent
        while (!_sendQueue.empty()) {
            PrioritizedEntity entry = _sendQueue.top();
            EntityItemPointer entity = entry.getEntity();
            if (entity) {
                std::cout << "adebug    send '" << entity->getName().toStdString() << "'" << "  :  " << entry.getPriority() << std::endl;  // adebug
            }
            _sendQueue.pop();
            _entitiesToSend.erase(entity);
        }
    }
#endif // SEND_SORTED_ENTITIES
    // END EXPERIMENTAL DIFFERENTIAL TRAVERSAL

    OctreeSendThread::traverseTreeAndSendContents(node, nodeData, viewFrustumChanged, isFullScene);
}

bool EntityTreeSendThread::addAncestorsToExtraFlaggedEntities(const QUuid& filteredEntityID,
                                                              EntityItem& entityItem, EntityNodeData& nodeData) {
    // check if this entity has a parent that is also an entity
    bool success = false;
    auto entityParent = entityItem.getParentPointer(success);

    if (success && entityParent && entityParent->getNestableType() == NestableType::Entity) {
        // we found a parent that is an entity item

        // first add it to the extra list of things we need to send
        bool parentWasNew = nodeData.insertFlaggedExtraEntity(filteredEntityID, entityParent->getID());

        // now recursively call ourselves to get its ancestors added too
        auto parentEntityItem = std::static_pointer_cast<EntityItem>(entityParent);
        bool ancestorsWereNew = addAncestorsToExtraFlaggedEntities(filteredEntityID, *parentEntityItem, nodeData);

        // return boolean if our parent or any of our ancestors were new additions (via insertFlaggedExtraEntity)
        return parentWasNew || ancestorsWereNew;
    }

    // since we didn't have a parent niether of our parents or ancestors could be new additions
    return false;
}

bool EntityTreeSendThread::addDescendantsToExtraFlaggedEntities(const QUuid& filteredEntityID,
                                                                EntityItem& entityItem, EntityNodeData& nodeData) {
    bool hasNewChild = false;
    bool hasNewDescendants = false;

    // enumerate the immediate children of this entity
    foreach (SpatiallyNestablePointer child, entityItem.getChildren()) {

        if (child && child->getNestableType() == NestableType::Entity) {
            // this is a child that is an entity

            // first add it to the extra list of things we need to send
            hasNewChild |= nodeData.insertFlaggedExtraEntity(filteredEntityID, child->getID());

            // now recursively call ourselves to get its descendants added too
            auto childEntityItem = std::static_pointer_cast<EntityItem>(child);
            hasNewDescendants |= addDescendantsToExtraFlaggedEntities(filteredEntityID, *childEntityItem, nodeData);
        }
    }

    // return our boolean indicating if we added new children or descendants as extra entities to send
    // (via insertFlaggedExtraEntity)
    return hasNewChild || hasNewDescendants;
}

void EntityTreeSendThread::startNewTraversal(const ViewFrustum& view, EntityTreeElementPointer root, int32_t lodLevelOffset) {
    DiffTraversal::Type type = _traversal.prepareNewTraversal(view, root, lodLevelOffset);
    // there are three types of traversal:
    //
    //      (1) FirstTime = at login --> find everything in view
    //      (2) Repeat = view hasn't changed --> find what has changed since last complete traversal
    //      (3) Differential = view has changed --> find what has changed or in new view but not old
    //
    // The "scanCallback" we provide to the traversal depends on the type:
    //
    // The _conicalView is updated here as a cached view approximation used by the lambdas for efficient
    // computation of entity sorting priorities.
    //
    _conicalView.set(_traversal.getCurrentView());

    switch (type) {
    case DiffTraversal::First:
        _traversal.setScanCallback([&] (DiffTraversal::VisibleElement& next) {
            next.element->forEachEntity([&](EntityItemPointer entity) {
                // Bail early if we've already checked this entity this frame
                if (_entitiesToSend.find(entity) != _entitiesToSend.end()) {
                    return;
                }
                bool success = false;
                AACube cube = entity->getQueryAACube(success);
                if (success) {
                    if (_traversal.getCurrentView().cubeIntersectsKeyhole(cube)) {
                        // Check the size of the entity, it's possible that a "too small to see" entity is included in a
                        // larger octree cell because of its position (for example if it crosses the boundary of a cell it
                        // pops to the next higher cell. So we want to check to see that the entity is large enough to be seen
                        // before we consider including it.
                        float renderAccuracy = calculateRenderAccuracy(_traversal.getCurrentView().getPosition(),
                                                                       cube,
                                                                       _traversal.getCurrentRootSizeScale(),
                                                                       _traversal.getCurrentLODOffset());

                        // Only send entities if they are large enough to see
                        if (renderAccuracy > 0.0f) {
                            float priority = _conicalView.computePriority(cube);
                            _sendQueue.push(PrioritizedEntity(entity, priority));
                            _entitiesToSend.insert(entity);
                        }
                    }
                } else {
                    const float WHEN_IN_DOUBT_PRIORITY = 1.0f;
                    _sendQueue.push(PrioritizedEntity(entity, WHEN_IN_DOUBT_PRIORITY));
                    _entitiesToSend.insert(entity);
                }
            });
        });
        break;
    case DiffTraversal::Repeat:
        _traversal.setScanCallback([&] (DiffTraversal::VisibleElement& next) {
            if (next.element->getLastChangedContent() > _traversal.getStartOfCompletedTraversal()) {
                uint64_t timestamp = _traversal.getStartOfCompletedTraversal();
                next.element->forEachEntity([&](EntityItemPointer entity) {
                    // Bail early if we've already checked this entity this frame
                    if (_entitiesToSend.find(entity) != _entitiesToSend.end()) {
                        return;
                    }
                    if (entity->getLastEdited() > timestamp) {
                        bool success = false;
                        AACube cube = entity->getQueryAACube(success);
                        if (success) {
                            if (next.intersection == ViewFrustum::INSIDE || _traversal.getCurrentView().cubeIntersectsKeyhole(cube)) {
                                // See the DiffTraversal::First case for an explanation of the "entity is too small" check
                                float renderAccuracy = calculateRenderAccuracy(_traversal.getCurrentView().getPosition(),
                                                                               cube,
                                                                               _traversal.getCurrentRootSizeScale(),
                                                                               _traversal.getCurrentLODOffset());

                                // Only send entities if they are large enough to see
                                if (renderAccuracy > 0.0f) {
                                    float priority = _conicalView.computePriority(cube);
                                    _sendQueue.push(PrioritizedEntity(entity, priority));
                                    _entitiesToSend.insert(entity);
                                }
                            }
                        } else {
                            const float WHEN_IN_DOUBT_PRIORITY = 1.0f;
                            _sendQueue.push(PrioritizedEntity(entity, WHEN_IN_DOUBT_PRIORITY));
                            _entitiesToSend.insert(entity);
                        }
                    }
                });
            }
        });
        break;
    case DiffTraversal::Differential:
        _traversal.setScanCallback([&] (DiffTraversal::VisibleElement& next) {
            // NOTE: for Differential case: next.intersection is against completedView not currentView
            uint64_t timestamp = _traversal.getStartOfCompletedTraversal();
            if (next.element->getLastChangedContent() > timestamp || next.intersection != ViewFrustum::INSIDE) {
                next.element->forEachEntity([&](EntityItemPointer entity) {
                    // Bail early if we've already checked this entity this frame
                    if (_entitiesToSend.find(entity) != _entitiesToSend.end()) {
                        return;
                    }
                    bool success = false;
                    AACube cube = entity->getQueryAACube(success);
                    if (success) {
                        if (_traversal.getCurrentView().cubeIntersectsKeyhole(cube)) {
                            // See the DiffTraversal::First case for an explanation of the "entity is too small" check
                            float renderAccuracy = calculateRenderAccuracy(_traversal.getCurrentView().getPosition(),
                                                                           cube,
                                                                           _traversal.getCurrentRootSizeScale(),
                                                                           _traversal.getCurrentLODOffset());

                            // Only send entities if they are large enough to see
                            if (renderAccuracy > 0.0f) {
                                if (entity->getLastEdited() > timestamp || !_traversal.getCompletedView().cubeIntersectsKeyhole(cube)) {
                                    float priority = _conicalView.computePriority(cube);
                                    _sendQueue.push(PrioritizedEntity(entity, priority));
                                    _entitiesToSend.insert(entity);
                                } else {
                                    // If this entity was skipped last time because it was too small, we still need to send it
                                    float lastRenderAccuracy = calculateRenderAccuracy(_traversal.getCompletedView().getPosition(),
                                                                                       cube,
                                                                                        _traversal.getCompletedRootSizeScale(),
                                                                                        _traversal.getCompletedLODOffset());

                                    if (lastRenderAccuracy <= 0.0f) {
                                        float priority = _conicalView.computePriority(cube);
                                        _sendQueue.push(PrioritizedEntity(entity, priority));
                                        _entitiesToSend.insert(entity);
                                    }
                                }
                            }
                        }
                    } else {
                        const float WHEN_IN_DOUBT_PRIORITY = 1.0f;
                        _sendQueue.push(PrioritizedEntity(entity, WHEN_IN_DOUBT_PRIORITY));
                        _entitiesToSend.insert(entity);
                    }
                });
            }
        });
        break;
    }
}

bool EntityTreeSendThread::traverseTreeAndBuildNextPacketPayload(EncodeBitstreamParams& params) {
#ifdef SEND_SORTED_ENTITIES
    //auto entityTree = std::static_pointer_cast<EntityTree>(_myServer->getOctree());
    if (_sendQueue.empty()) {
        return false;
    }
    if (!_packetData.hasContent()) {
        // This is the beginning of a new packet.
        // We pack minimal data for this to be accepted as an OctreeElement payload for the root element.
        // The Octree header bytes look like this:
        //
        // 0x00  octalcode for root
        // 0x00  colors (1 bit where recipient should call: child->readElementDataFromBuffer())
        // 0xXX  childrenInTreeMask (when params.includeExistsBits is true: 1 bit where child is existant)
        // 0x00  childrenInBufferMask (1 bit where recipient should call: child->readElementData() recursively)
        const uint8_t zeroByte = 0;
        _packetData.appendValue(zeroByte); // octalcode
        _packetData.appendValue(zeroByte); // colors
        if (params.includeExistsBits) {
            uint8_t childrenExistBits = 0;
            EntityTreeElementPointer root = std::dynamic_pointer_cast<EntityTreeElement>(_myServer->getOctree()->getRoot());
            for (int32_t i = 0; i < NUMBER_OF_CHILDREN; ++i) {
                if (root->getChildAtIndex(i)) {
                    childrenExistBits += (1 << i);
                }
            }
            _packetData.appendValue(childrenExistBits); // childrenInTreeMask
        }
        _packetData.appendValue(zeroByte); // childrenInBufferMask

        // Pack zero for numEntities.
        // But before we do: grab current byteOffset so we can come back later
        // and update this with the real number.
        _numEntities = 0;
        _numEntitiesOffset = _packetData.getUncompressedByteOffset();
        _packetData.appendValue(_numEntities);
    }

    LevelDetails entitiesLevel = _packetData.startLevel();
    while(!_sendQueue.empty()) {
        PrioritizedEntity queuedItem = _sendQueue.top();
        EntityItemPointer entity = queuedItem.getEntity();
        if (entity) {
            OctreeElement::AppendState appendEntityState = entity->appendEntityData(&_packetData, params, _extraEncodeData);

            if (appendEntityState != OctreeElement::COMPLETED) {
                if (appendEntityState == OctreeElement::PARTIAL) {
                    ++_numEntities;
                }
                params.stopReason = EncodeBitstreamParams::DIDNT_FIT;
                break;
            }
            ++_numEntities;
        }
        _sendQueue.pop();
    }
    if (_sendQueue.empty()) {
        params.stopReason = EncodeBitstreamParams::FINISHED;
        _extraEncodeData->entities.clear();
    }

    if (_numEntities == 0) {
        _packetData.discardLevel(entitiesLevel);
        return false;
    }
    _packetData.endLevel(entitiesLevel);
    _packetData.updatePriorBytes(_numEntitiesOffset, (const unsigned char*)&_numEntities, sizeof(_numEntities));
    return true;

#else // SEND_SORTED_ENTITIES
    return OctreeSendThread::traverseTreeAndBuildNextPacketPayload(params);
#endif // SEND_SORTED_ENTITIES
}

