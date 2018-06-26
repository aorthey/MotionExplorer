#pragma once
#include "elements/hierarchy.h"
#include "elements/roadmap.h"
typedef Hierarchy<RoadmapPtr> HierarchicalRoadmap;
typedef std::shared_ptr<HierarchicalRoadmap> HierarchicalRoadmapPtr;
