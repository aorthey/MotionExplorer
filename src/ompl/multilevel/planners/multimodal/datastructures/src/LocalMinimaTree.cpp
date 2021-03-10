#include "util.h"
#include <ompl/multilevel/planners/multimodal/datastructures/LocalMinimaTree.h>
#include <ompl/util/Console.h>
#include <thread>
#include <ompl/geometric/PathSimplifier.h>

using namespace ompl::multilevel;

int LocalMinimaNode::id_counter = 0;

LocalMinimaTree::LocalMinimaTree(std::vector<base::SpaceInformationPtr> siVec) : siVec_(siVec)
{
    OMPL_DEBUG("Number of threads supported by implementation: %d", 
        std::thread::hardware_concurrency());
    for (uint k = 0; k < siVec.size(); k++)
    {
        std::vector<LocalMinimaNode *> kthLevel;
        tree_.push_back(kthLevel);
    }
}

LocalMinimaTree::~LocalMinimaTree()
{
}

void LocalMinimaTree::clear()
{
    hasChanged_ = false;
    selectedMinimum_.clear();

    for (uint k = 0; k < tree_.size(); k++)
    {
        std::vector<LocalMinimaNode *> &treeLevelk = tree_.at(k);
        for (uint j = 0; j < treeLevelk.size(); j++)
        {
            LocalMinimaNode *node = treeLevelk.at(j);
            delete node;
            treeLevelk.at(j) = nullptr;
        }
        treeLevelk.clear();
    }

    numberOfMinima_ = 0;
    levels_ = 0;
}

unsigned int LocalMinimaTree::getNumberOfMinima(unsigned int level) const
{
    return tree_.at(level).size();
}

unsigned int LocalMinimaTree::getNumberOfMinima() const
{
    return numberOfMinima_;
}

unsigned int LocalMinimaTree::getNumberOfLevel() const
{
    return tree_.size();
}

std::vector<int> LocalMinimaTree::getSelectedPathIndex() const
{
    return selectedMinimum_;
}

void LocalMinimaTree::setSelectedPathIndex(std::vector<int> selectedMinimum)
{
    std::lock_guard<std::recursive_mutex> guard(getLock());
    selectedMinimum_ = selectedMinimum;
}

std::recursive_mutex &LocalMinimaTree::getLock()
{
    return lock_;
}

unsigned int LocalMinimaTree::getNumberOfLevelContainingMinima() const
{
    unsigned int ctr = 0;

    while (ctr <= (tree_.size() - 1) && tree_.at(ctr).size() > 0)
    {
        ctr++;
    }
    return ctr;
}

bool LocalMinimaTree::hasChanged()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());
    if (hasChanged_)
    {
        hasChanged_ = false;
        return true;
    }
    else
    {
        return false;
    }
}

bool LocalMinimaTree::isConverged() const
{
    const LocalMinimaNode *node = getSelectedPath();
    if(node==nullptr || !node->isConverged()) return false;

    std::vector<LocalMinimaNode *> nodes = getSelectedPathSiblings();
    for(uint k = 0; k < nodes.size(); k++)
    {
      LocalMinimaNode *node = nodes.at(k);
      if(!node->isConverged()) return false;
    }
    return true;
}

bool LocalMinimaNode::isConverged() const
{
    return (numberOfIdempotentUpdates_ > 2);
}

LocalMinimaNode *LocalMinimaTree::updatePath(
    base::PathPtr path, double cost, int level, int index)
{
    std::lock_guard<std::recursive_mutex> guard(getLock());
    sanityCheckLevelIndex(level, index);

    LocalMinimaNode *node = tree_.at(level).at(index);
    node->setPathPtr(path);
    node->setLevel(level);
    // node->update();

    double costOld = node->getCost();
    node->setCost(cost);

    if(cost < costOld)
    {
        node->numberOfIdempotentUpdates_ = 0;
    }else
    {
        node->numberOfIdempotentUpdates_++;
    }

    hasChanged_ = true;

    if ((int)selectedMinimum_.size() == level - 1)
    {
        selectedMinimum_.back() = index;
    }
    return node;
}

LocalMinimaNode *LocalMinimaTree::addPath(
    base::PathPtr path, double cost, int level)
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    // base::OptimizationObjectivePtr obj =
    //   std::make_shared<ompl::base::PathLengthOptimizationObjective>(siVec_.at(level));
    // geometric::PathSimplifier shortcutter(siVec_.at(level), base::GoalPtr(), obj);

    // geometric::PathGeometricPtr gpath = std::dynamic_pointer_cast<geometric::PathGeometric>(path);
    // if (gpath != nullptr)
    // {
    //     // gpath->interpolate();
    // }

    LocalMinimaNode *node = new LocalMinimaNode(siVec_.at(level), path);
    node->setLevel(level);
    node->setCost(cost);
    // node->update();

    tree_.at(level).push_back(node);
    numberOfMinima_++;
    hasChanged_ = true;

    setSelectedMinimumExpand();
    selectedMinimum_.back() = tree_.at(level).size() - 1;
    return node;
}

void LocalMinimaTree::removePath(int level, int index)
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    LocalMinimaNode *node = tree_.at(level).at(index);
    if(node->customRepresentation) delete node->customRepresentation;
    if(node) delete node;

    tree_.at(level).erase(tree_.at(level).begin() + index);
    numberOfMinima_--;
    hasChanged_ = true;

    setSelectedMinimumNext();
    setSelectedMinimumPrev();
}

LocalMinimaNode *LocalMinimaTree::getPath(int level, int index) const
{
    sanityCheckLevelIndex(level, index);
    return tree_.at(level).at(index);
}

LocalMinimaNode *LocalMinimaTree::getSelectedPath() const
{
    int level = selectedMinimum_.size() - 1;
    if (level < 0)
        return nullptr;

    int index = selectedMinimum_.back();
    return getPath(level, index);
}

std::vector<LocalMinimaNode *> LocalMinimaTree::getPaths(int level) const
{
    std::vector<LocalMinimaNode *> nodeVector;
    for (uint index = 0; index < getNumberOfMinima(level); index++)
    {
        LocalMinimaNode *nodek = getPath(level, index);
        nodeVector.push_back(nodek);
    }
    return nodeVector;
}

std::vector<LocalMinimaNode *> LocalMinimaTree::getSelectedPathSiblings() const
{
    std::vector<LocalMinimaNode *> nodeVector;

    int level = selectedMinimum_.size() - 1;
    if (level < 0)
        return nodeVector;

    int indexSelectedPath = selectedMinimum_.back();

    // std::cout << level << " has " << getNumberOfMinima(level) << " paths." << std::endl;
    for (uint k = 0; k < getNumberOfMinima(level); k++)
    {
        if ((int)k == indexSelectedPath)
            continue;
        LocalMinimaNode *nodek = getPath(level, k);
        nodeVector.push_back(nodek);
    }
    return nodeVector;
}

void LocalMinimaTree::sanityCheckLevelIndex(int level, int index) const
{
    if (level < 0 || level > (int)getNumberOfLevel() - 1)
    {
        OMPL_ERROR("Tried getting path cost for non existing path.");
        throw ompl::Exception("Nonexisting");
    }
    if (index < 0 || index > (int)getNumberOfMinima(level) - 1)
    {
        OMPL_ERROR("Tried getting path cost for non existing path.");
        throw ompl::Exception("Nonexisting");
    }
}

double LocalMinimaTree::getPathCost(int level, int index) const
{
    sanityCheckLevelIndex(level, index);
    LocalMinimaNode *node = tree_.at(level).at(index);
    return node->getCost();
}

void LocalMinimaTree::printSelectedMinimum()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());
    if (selectedMinimum_.size() > 0)
    {
        int level = selectedMinimum_.size() - 1;
        int maxMinima = getNumberOfMinima(selectedMinimum_.size() - 1);
        OMPL_DEVMSG1("Selected local minimum %d/%d (level %d, cost %.2f%s)", 
          selectedMinimum_.back() + 1, 
          maxMinima, 
          level,
          tree_.at(level).at(selectedMinimum_.back())->getCost(),
          (tree_.at(level).at(selectedMinimum_.back())->isConverged()?
           ", [converged]":""));
    }
}

void LocalMinimaTree::setSelectedMinimumPrev()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    if (selectedMinimum_.size() > 0)
    {
        int maxMinima = getNumberOfMinima(selectedMinimum_.size() - 1);
        if (maxMinima > 0)
        {
            if (selectedMinimum_.back() > 0)
            {
                selectedMinimum_.back()--;
            }
            else
            {
                selectedMinimum_.back() = maxMinima - 1;
            }
        }
    }
    hasChanged_ = true;
}


void LocalMinimaTree::setSelectedMinimumNext()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    if (selectedMinimum_.size() > 0)
    {
        int maxMinima = getNumberOfMinima(selectedMinimum_.size() - 1);
        if (maxMinima > 0)
        {
            if (selectedMinimum_.back() < maxMinima - 1)
            {
                selectedMinimum_.back()++;
            }else{
                selectedMinimum_.back() = 0;
            }
        }
    }
    hasChanged_ = true;
}

void LocalMinimaTree::setSelectedMinimumCollapse()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    if (selectedMinimum_.size() > 0)
    {
        selectedMinimum_.erase(selectedMinimum_.end() - 1);
    }
    hasChanged_ = true;
}

void LocalMinimaTree::setSelectedMinimumExpand()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    unsigned int maxLevel = getNumberOfLevelContainingMinima();
    if (selectedMinimum_.size() < maxLevel)
    {
        selectedMinimum_.push_back(0);
    }
    hasChanged_ = true;
}

void LocalMinimaTree::setSelectedMinimumExpandFull()
{
    std::lock_guard<std::recursive_mutex> guard(getLock());

    unsigned int maxLevel = getNumberOfLevelContainingMinima();
    while (selectedMinimum_.size() < maxLevel)
    {
        selectedMinimum_.push_back(0);
    }
    hasChanged_ = true;
}

// void LocalMinimaTree::printSelectedMinimum()
// {
//     for (uint k = 0; k < selectedMinimum_.size(); k++)
//     {
//         std::cout << selectedMinimum_.at(k);
//         std::cout << (k < selectedMinimum_.size() - 1 ? " > " : "");
//     }
//     std::cout << std::endl;
// }

const std::vector<ompl::base::State *>&
LocalMinimaTree::getSelectedMinimumAsStateVector(int level) const
{
    int selectedMinimumOnLevel = selectedMinimum_.at(level);
    LocalMinimaNode *node = tree_.at(level).at(selectedMinimumOnLevel);
    return node->asStates();
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

void LocalMinimaNode::init()
{
    id_ = id_counter++;
    // export_ = true;
    // timePointStart = ompl::time::now();

    // if(export_)
    // {
    //     xmlNode_ = CreateRootNodeInDocument(xmlDoc_);
    //     xmlNode_->SetValue("path");
    //     AddComment(*xmlNode_, "This file contains the evolution of a path over time. A path is represented by a sequence of states and a time index.");
    // }
}
// void LocalMinimaNode::update()
// {
//     if(export_)
//     {

//         saveXML(xmlNode_);
//         // TiXmlDocument xmlDoc;
//         // xmlDoc_.LinkEndChild(xmlNode_);
//         // std::string fn = "curve"+std::to_string(id_);
//         // xmlDoc_.SaveFile(fn);
//     }
// }
LocalMinimaNode::~LocalMinimaNode()
{
    // finish();
}
//void LocalMinimaNode::finish()
//{
//    if(export_)
//    {
//        saveXML(xmlNode_);
//        xmlDoc_.LinkEndChild(xmlNode_);
//        std::string fn = "disk_curve"+std::to_string(id_)+".path";

//        xmlDoc_.SaveFile(fn);
//        std::cout << "SAVE curve file to " << fn << std::endl;
//    }
//}

//void LocalMinimaNode::saveXML(TiXmlElement *root_node)
//{
//    //Add time
//    timePointEnd = ompl::time::now();
//    ompl::time::duration timeDuration = timePointEnd - timePointStart;
//    double duration = ompl::time::seconds(timeDuration);
//    int timeInt = int(duration*25.0);
//    std::string time = std::to_string(timeInt);

//    bool timeExist = (std::find(updateTimes_.begin(), updateTimes_.end(), timeInt) != updateTimes_.end());
//    if(timeExist) return;
//    updateTimes_.push_back(timeInt);

//    TiXmlElement* node = new TiXmlElement("keyframe");

//    node->SetAttribute("time", time);

//    if(hasStatesRepresentation)
//    {
//        unsigned int N = spath_.size();
//        for(uint k = 0; k < N; k++)
//        {
//            TiXmlElement* subnode = new TiXmlElement("state");

//            base::StateSpacePtr space = path_->getSpaceInformation()->getStateSpace();
//            std::vector<double> state_serialized;
//            base::State *state = spath_.at(k);
//            space->copyToReals(state_serialized, state);
//            std::stringstream ss;
//            ss << state_serialized.size() << "  ";
//            for(uint k = 0; k < state_serialized.size(); k++){
//              ss << state_serialized.at(k) << " ";
//            }
//            TiXmlText text(ss.str().c_str());
//            subnode->InsertEndChild(text);
//            node->InsertEndChild(*subnode);
//        }
//    }


//    root_node->InsertEndChild(*node);
//}
