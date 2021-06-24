#include "util.h"
#include <ompl/multilevel/planners/multimodal/datastructures/LocalMinimaNode.h>
#include <ompl/util/Console.h>
#include <thread>

using namespace ompl::multilevel;

int LocalMinimaNode::id_counter = 0;

LocalMinimaNode::LocalMinimaNode(base::SpaceInformationPtr si, StatesPath &states)
{
    si_ = si;
    spath_ = states;
    hasStatesRepresentation = true;
    init();
}

LocalMinimaNode::LocalMinimaNode(base::SpaceInformationPtr si, VertexPath &vertices)
{
    si_ = si;
    vpath_ = vertices;
    hasVertexRepresentation = true;
    init();
}

LocalMinimaNode::LocalMinimaNode(base::SpaceInformationPtr si, base::PathPtr path)
{
    si_ = si;
    path_ = path;
    geometric::PathGeometricPtr gpath = 
      std::dynamic_pointer_cast<geometric::PathGeometric>(path);
    if (gpath != nullptr)
    {
      spath_ = gpath->getStates();
      hasStatesRepresentation = true;
    }
    hasPathPtrRepresentation = true;
    init();
}

double LocalMinimaNode::getCost() const
{
    return cost_.value();
}

void LocalMinimaNode::setVertexPath(VertexPath &vertices)
{
    vpath_ = vertices;
    hasVertexRepresentation = true;
}

void LocalMinimaNode::setPathPtr(base::PathPtr path)
{
    path_ = path;
    hasPathPtrRepresentation = true;

    geometric::PathGeometricPtr gpath = 
      std::static_pointer_cast<geometric::PathGeometric>(path);
    spath_ = gpath->getStates();
    hasStatesRepresentation = true;
}

void LocalMinimaNode::init()
{
    id_ = id_counter++;
}

LocalMinimaNode::~LocalMinimaNode()
{
}

void LocalMinimaNode::setNsubtresholdIterations(int N)
{
    NsubtresholdIterations_ = N;
}

bool LocalMinimaNode::isConverged() const
{
    return (numberOfIdempotentUpdates_ > 10);
}

const ompl::base::PathPtr &LocalMinimaNode::asPathPtr() const
{
    if (hasPathPtrRepresentation)
        return path_;

    OMPL_ERROR("NYI");
    throw ompl::Exception("NYI");
}

ompl::base::PathPtr &LocalMinimaNode::asPathPtrNonConst()
{
    if (hasPathPtrRepresentation)
        return path_;

    OMPL_ERROR("NYI");
    throw ompl::Exception("NYI");
}

const StatesPath &LocalMinimaNode::asStates() const
{
    if (hasStatesRepresentation)
        return spath_;

    OMPL_ERROR("NYI");
    throw ompl::Exception("NYI");
}

StatesPath &LocalMinimaNode::asStatesNonConst() 
{
    if (hasStatesRepresentation)
        return spath_;

    OMPL_ERROR("NYI");
    throw ompl::Exception("NYI");
}

const VertexPath &LocalMinimaNode::asVertices() const
{
    if (hasVertexRepresentation)
        return vpath_;
    OMPL_ERROR("NYI");
    throw ompl::Exception("NYI");
}

void LocalMinimaNode::setCost(double c)
{
    numberOfCostUpdates_++;
    cost_ = base::Cost(c);
}

int LocalMinimaNode::getLevel() const
{
    return level_;
}

void LocalMinimaNode::setLevel(int level)
{
    level_ = level;
}

int LocalMinimaNode::getId() const
{
    return id_;
}
int LocalMinimaNode::getNumberOfCostUpdates()
{
    return numberOfCostUpdates_;
}
