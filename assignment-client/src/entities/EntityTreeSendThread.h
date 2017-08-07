//
//  EntityTreeSendThread.h
//  assignment-client/src/entities
//
//  Created by Stephen Birarda on 2/15/17.
//  Copyright 2017 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#ifndef hifi_EntityTreeSendThread_h
#define hifi_EntityTreeSendThread_h

#include <queue>

#include "../octree/OctreeSendThread.h"

#include <AACube.h>

#include "EntityTreeElement.h"

const float SQRT_TWO_OVER_TWO = 0.7071067811865;
const float DEFAULT_VIEW_RADIUS = 10.0f;

class EntityNodeData;
class EntityItem;


class TreeTraversalPath {
public:
    class ConicalView {
    public:
        ConicalView() {}
        ConicalView(const ViewFrustum& viewFrustum) { set(viewFrustum); }
        void set(const ViewFrustum& viewFrustum);
        float computePriority(const AACube& cube) const;
        float computePriority(const EntityItemPointer& entity) const;
    private:
        glm::vec3 _position { 0.0f, 0.0f, 0.0f };
        glm::vec3 _direction { 0.0f, 0.0f, 1.0f };
        float _sinAngle { SQRT_TWO_OVER_TWO };
        float _cosAngle { SQRT_TWO_OVER_TWO };
        float _radius { DEFAULT_VIEW_RADIUS };
    };

    class PrioritizedEntity {
    public:
        PrioritizedEntity(EntityItemPointer entity, float priority) : _weakEntity(entity), _priority(priority) { }
        float updatePriority(const ConicalView& view);
        EntityItemPointer getEntity() const { return _weakEntity.lock(); }
        float getPriority() const { return _priority; }

        class Compare {
        public:
            bool operator() (const PrioritizedEntity& A, const PrioritizedEntity& B) { return A._priority < B._priority; }
        };
        friend class Compare;

    private:
        EntityItemWeakPointer _weakEntity;
        float _priority;
    };

    class Fork {
    public:
        Fork(EntityTreeElementPointer& element);

        EntityTreeElementPointer getNextElementFirstTime(const ViewFrustum& view);
        EntityTreeElementPointer getNextElementAgain(const ViewFrustum& view, uint64_t lastTime);
        EntityTreeElementPointer getNextElementDifferential(const ViewFrustum& view, const ViewFrustum& lastView, uint64_t lastTime);

        int8_t getNextIndex() const { return _nextIndex; }
        void initRootNextIndex() { _nextIndex = -1; }

    protected:
        EntityTreeElementWeakPointer _weakElement;
        int8_t _nextIndex;
    };

    TreeTraversalPath();

    void startNewTraversal(const ViewFrustum& viewFrustum, EntityTreeElementPointer root);

    EntityTreeElementPointer getNextElement();

    const ViewFrustum& getView() const { return _currentView; }

    bool empty() const { return _forks.empty(); }
    size_t size() const { return _forks.size(); } // adebug
    void dump() const;

    const ViewFrustum& getCurrentView() const { return _currentView; }

protected:
    ViewFrustum _currentView;
    ViewFrustum _completedView;
    std::vector<Fork> _forks;
    std::function<EntityTreeElementPointer()> _getNextElementCallback { nullptr };
    uint64_t _startOfCompletedTraversal { 0 };
    uint64_t _startOfCurrentTraversal { 0 };
};

using EntityPriorityQueue = std::priority_queue< TreeTraversalPath::PrioritizedEntity, std::vector<TreeTraversalPath::PrioritizedEntity>, TreeTraversalPath::PrioritizedEntity::Compare >;


class EntityTreeSendThread : public OctreeSendThread {

public:
    EntityTreeSendThread(OctreeServer* myServer, const SharedNodePointer& node) : OctreeSendThread(myServer, node) {};

protected:
    void preDistributionProcessing() override;
    void traverseTreeAndSendContents(SharedNodePointer node, OctreeQueryNode* nodeData,
            bool viewFrustumChanged, bool isFullScene) override;

private:
    // the following two methods return booleans to indicate if any extra flagged entities were new additions to set
    bool addAncestorsToExtraFlaggedEntities(const QUuid& filteredEntityID, EntityItem& entityItem, EntityNodeData& nodeData);
    bool addDescendantsToExtraFlaggedEntities(const QUuid& filteredEntityID, EntityItem& entityItem, EntityNodeData& nodeData);

    TreeTraversalPath _path;
    EntityPriorityQueue _sendQueue;
};

#endif // hifi_EntityTreeSendThread_h
