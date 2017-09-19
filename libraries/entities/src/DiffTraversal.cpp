//
//  DiffTraversal.cpp
//  assignment-client/src/entities
//
//  Created by Andrew Meadows 2017.08.08
//  Copyright 2017 High Fidelity, Inc.
//
//  Distributed under the Apache License, Version 2.0.
//  See the accompanying file LICENSE or http://www.apache.org/licenses/LICENSE-2.0.html
//

#include "DiffTraversal.h"

#include <OctreeUtils.h>


DiffTraversal::Waypoint::Waypoint(EntityTreeElementPointer& element) : _nextIndex(0) {
    assert(element);
    _weakElement = element;
}

void DiffTraversal::Waypoint::getNextVisibleElementFirstTime(DiffTraversal::VisibleElement& next,
        const DiffTraversal::View& view) {
    // NOTE: no need to set next.intersection in the "FirstTime" context
    if (_nextIndex == -1) {
        // root case is special:
        // its intersection is always INTERSECT,
        // we never bother checking for LOD culling, and
        // we can skip it if the content hasn't changed
        ++_nextIndex;
        next.element = _weakElement.lock();
        return;
    } else if (_nextIndex < NUMBER_OF_CHILDREN) {
        EntityTreeElementPointer element = _weakElement.lock();
        if (element) {
            while (_nextIndex < NUMBER_OF_CHILDREN) {
                EntityTreeElementPointer nextElement = element->getChildAtIndex(_nextIndex);
                ++_nextIndex;
                if (nextElement) {
                    if (!view.usesViewFrustum) {
                        // No LOD truncation if we aren't using the view frustum
                        next.element = nextElement;
                        return;
                    } else if (view.viewFrustum.cubeIntersectsKeyhole(nextElement->getAACube())) {
                        // check for LOD truncation
                        float distance = glm::distance(view.viewFrustum.getPosition(), nextElement->getAACube().calcCenter()) + MIN_VISIBLE_DISTANCE;
                        float apparentAngle = nextElement->getAACube().getScale() / distance;
                        if (apparentAngle > MIN_ELEMENT_APPARENT_ANGLE * view.lodScaleFactor) {
                            next.element = nextElement;
                            return;
                        }
                    }
                }
            }
        }
    }
    next.element.reset();
}

void DiffTraversal::Waypoint::getNextVisibleElementRepeat(
        DiffTraversal::VisibleElement& next, const DiffTraversal::View& view, uint64_t lastTime) {
    if (_nextIndex == -1) {
        // root case is special
        ++_nextIndex;
        EntityTreeElementPointer element = _weakElement.lock();
        if (element->getLastChangedContent() > lastTime) {
            next.element = element;
            next.intersection = ViewFrustum::INTERSECT;
            return;
        }
    }
    if (_nextIndex < NUMBER_OF_CHILDREN) {
        EntityTreeElementPointer element = _weakElement.lock();
        if (element) {
            while (_nextIndex < NUMBER_OF_CHILDREN) {
                EntityTreeElementPointer nextElement = element->getChildAtIndex(_nextIndex);
                ++_nextIndex;
                if (nextElement && nextElement->getLastChanged() > lastTime) {
                    if (!view.usesViewFrustum) {
                        // No LOD truncation if we aren't using the view frustum
                        next.element = nextElement;
                        next.intersection = ViewFrustum::INSIDE;
                        return;
                    } else {
                        // check for LOD truncation
                        float distance = glm::distance(view.viewFrustum.getPosition(), nextElement->getAACube().calcCenter()) + MIN_VISIBLE_DISTANCE;
                        float apparentAngle = nextElement->getAACube().getScale() / distance;
                        if (apparentAngle > MIN_ELEMENT_APPARENT_ANGLE * view.lodScaleFactor) {
                            ViewFrustum::intersection intersection = view.viewFrustum.calculateCubeKeyholeIntersection(nextElement->getAACube());
                            if (intersection != ViewFrustum::OUTSIDE) {
                                next.element = nextElement;
                                next.intersection = intersection;
                                return;
                            }
                        }
                    }
                }
            }
        }
    }
    next.element.reset();
    next.intersection = ViewFrustum::OUTSIDE;
}

void DiffTraversal::Waypoint::getNextVisibleElementDifferential(DiffTraversal::VisibleElement& next,
        const DiffTraversal::View& view, const DiffTraversal::View& lastView, const bool fullTraversal) {
    if (_nextIndex == -1) {
        // root case is special
        ++_nextIndex;
        EntityTreeElementPointer element = _weakElement.lock();
        next.element = element;
        next.intersection = ViewFrustum::INTERSECT;
        return;
    } else if (_nextIndex < NUMBER_OF_CHILDREN) {
        EntityTreeElementPointer element = _weakElement.lock();
        if (element) {
            while (_nextIndex < NUMBER_OF_CHILDREN) {
                EntityTreeElementPointer nextElement = element->getChildAtIndex(_nextIndex);
                ++_nextIndex;
                if (nextElement) {
                    AACube cube = nextElement->getAACube();
                    // check for LOD truncation
                    float distance = glm::distance(view.viewFrustum.getPosition(), cube.calcCenter()) + MIN_VISIBLE_DISTANCE;
                    float apparentAngle = cube.getScale() / distance;
                    if (apparentAngle > MIN_ELEMENT_APPARENT_ANGLE * view.lodScaleFactor) {
                        if (view.viewFrustum.cubeIntersectsKeyhole(cube)) {
                            ViewFrustum::intersection lastIntersection = lastView.viewFrustum.calculateCubeKeyholeIntersection(cube);
                            // If an element is in view and was not in view before, we need to scan it
                            // If an element has changed, either it or one of its children may have changed, so we need to traverse it,
                            // but we will only need to scan it if it was this cell that changed
                            if (lastIntersection != ViewFrustum::INSIDE || nextElement->getLastChanged() > lastView.startTime) {
                                next.element = nextElement;
                                // NOTE: for differential case next.intersection is against the lastView
                                // because this helps the "external scan" optimize its culling
                                next.intersection = lastIntersection;
                                return;
                            } else {
                                distance = glm::distance(lastView.viewFrustum.getPosition(), cube.calcCenter()) + MIN_VISIBLE_DISTANCE;
                                apparentAngle = cube.getScale() / distance;
                                if (!fullTraversal) {
                                    // check for LOD truncation in the last traversal because
                                    // we may need to traverse this element after all if the lastView skipped it for LOD
                                    if (apparentAngle <= MIN_ELEMENT_APPARENT_ANGLE * lastView.lodScaleFactor) {
                                        next.element = nextElement;
                                        // element's intersection with lastView was effectively OUTSIDE
                                        next.intersection = ViewFrustum::OUTSIDE;
                                        return;
                                    }
                                } else {
                                    // If it's a fullTraversal, we need to traverse all the children, but we may not need to scan them
                                    // If we could see this element before, we don't need to scan it, so we mark it as INSIDE
                                    next.element = nextElement;
                                    next.intersection = apparentAngle > MIN_ELEMENT_APPARENT_ANGLE * lastView.lodScaleFactor ? ViewFrustum::INSIDE : ViewFrustum::OUTSIDE;
                                    return;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    next.element.reset();
    next.intersection = ViewFrustum::OUTSIDE;
}

DiffTraversal::DiffTraversal() {
    const int32_t MIN_PATH_DEPTH = 16;
    _path.reserve(MIN_PATH_DEPTH);
}

DiffTraversal::Type DiffTraversal::prepareNewTraversal(const ViewFrustum& viewFrustum, EntityTreeElementPointer root, int32_t lodLevelOffset, bool usesViewFrustum) {
    assert(root);
    // there are three types of traversal:
    //
    //   (1) First = fresh view --> find all elements in view
    //   (2) Repeat = view hasn't changed --> find elements changed since last complete traversal
    //   (3) Differential = view has changed --> find elements changed or in new view but not old
    //
    // for each traversal type we assign the appropriate _getNextVisibleElementCallback
    //
    //   _getNextVisibleElementCallback = identifies elements that need to be traversed,
    //        updates VisibleElement ref argument with pointer-to-element and view-intersection
    //        (INSIDE, INTERSECT, or OUTSIDE)
    //
    // external code should update the _scanElementCallback after calling prepareNewTraversal
    //
    _currentView.usesViewFrustum = usesViewFrustum;
    float lodScaleFactor = powf(2.0f, lodLevelOffset);

    Type type;
    // If usesViewFrustum changes, treat it as a First traversal
    if (_completedView.startTime == 0 || _currentView.usesViewFrustum != _completedView.usesViewFrustum) {
        type = Type::First;
        _currentView.viewFrustum = viewFrustum;
        _currentView.lodScaleFactor = lodScaleFactor;
        _getNextVisibleElementCallback = [&](DiffTraversal::VisibleElement& next) {
            _path.back().getNextVisibleElementFirstTime(next, _currentView);
        };
    } else if (!_currentView.usesViewFrustum ||
               (_completedView.viewFrustum.isVerySimilar(viewFrustum) &&
                lodScaleFactor == _completedView.lodScaleFactor)) {
        type = Type::Repeat;
        _getNextVisibleElementCallback = [&](DiffTraversal::VisibleElement& next) {
            _path.back().getNextVisibleElementRepeat(next, _completedView, _completedView.startTime);
        };
    } else {
        type = Type::Differential;
        _currentView.viewFrustum = viewFrustum;
        _currentView.lodScaleFactor = lodScaleFactor;
        bool fullTraversal = !_completedView.viewFrustum.isPositionVerySimilar(viewFrustum) ||
            lodScaleFactor < _completedView.lodScaleFactor;
        _getNextVisibleElementCallback = [&](DiffTraversal::VisibleElement& next) {
            // If the view's position changed or the LOD scale factor decreased, it's possible that a child
            // element is now close enough to not be LOD culled, so we need to we need to traverse all children
            // of in view elements, but we might not need to scan them all
            _path.back().getNextVisibleElementDifferential(next, _currentView, _completedView, fullTraversal);
        };
    }

    _path.clear();
    _path.push_back(DiffTraversal::Waypoint(root));
    // set root fork's index such that root element returned at getNextElement()
    _path.back().initRootNextIndex();

    _currentView.startTime = usecTimestampNow();

    return type;
}

void DiffTraversal::getNextVisibleElement(DiffTraversal::VisibleElement& next) {
    if (_path.empty()) {
        next.element.reset();
        next.intersection = ViewFrustum::OUTSIDE;
        return;
    }
    _getNextVisibleElementCallback(next);
    if (next.element) {
        int8_t nextIndex = _path.back().getNextIndex();
        if (nextIndex > 0) {
            _path.push_back(DiffTraversal::Waypoint(next.element));
        }
    } else {
        // we're done at this level
        while (!next.element) {
            // pop one level
            _path.pop_back();
            if (_path.empty()) {
                // we've traversed the entire tree
                _completedView = _currentView;
                return;
            }
            // keep looking for next
            _getNextVisibleElementCallback(next);
            if (next.element) {
                // we've descended one level so add it to the path
                _path.push_back(DiffTraversal::Waypoint(next.element));
            }
        }
    }
}

void DiffTraversal::setScanCallback(std::function<void (DiffTraversal::VisibleElement&)> cb) {
    if (!cb) {
        _scanElementCallback = [](DiffTraversal::VisibleElement& a){};
    } else {
        _scanElementCallback = cb;
    }
}

// DEBUG method: delete later
std::ostream& operator<<(std::ostream& s, const DiffTraversal& traversal) {
    for (size_t i = 0; i < traversal._path.size(); ++i) {
        s << (int32_t)(traversal._path[i].getNextIndex());
        if (i < traversal._path.size() - 1) {
            s << ":";
        }
    }
    return s;
}

void DiffTraversal::traverse(uint64_t timeBudget) {
    uint64_t expiry = usecTimestampNow() + timeBudget;
    DiffTraversal::VisibleElement next;
    getNextVisibleElement(next);
    while (next.element) {
        if (next.element->hasContent()) {
            _scanElementCallback(next);
        }
        if (usecTimestampNow() > expiry) {
            break;
        }
        getNextVisibleElement(next);
    }
}
