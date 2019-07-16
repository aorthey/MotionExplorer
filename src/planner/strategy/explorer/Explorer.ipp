#include "elements/plannerdata_vertex_annotated.h"
#include "QuotientGraphSparse.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/util/Time.h>
#include <queue>

using namespace og;
using namespace ob;

template <class T>
MotionExplorer<T>::MotionExplorer(std::vector<ob::SpaceInformationPtr> &siVec, std::string type)
  : ob::Planner(siVec.back(), type), siVec_(siVec)
{
    T::resetCounter();
    for (unsigned int k = 0; k < siVec_.size(); k++)
    {
        og::Quotient *parent = nullptr;
        if (k > 0)
            parent = quotientSpaces_.back();

        T *ss = new T(siVec_.at(k), parent);
        quotientSpaces_.push_back(ss);
        quotientSpaces_.back()->SetLevel(k);
    }
    if (DEBUG)
        std::cout << "Created hierarchy with " << siVec_.size() << " levels." << std::endl;
}

template <class T>
MotionExplorer<T>::~MotionExplorer()
{
}

template <class T>
void MotionExplorer<T>::setup()
{
    Planner::setup();
    for (unsigned int k = 0; k < quotientSpaces_.size(); k++)
    {
        quotientSpaces_.at(k)->setup();
    }
}

template <class T>
void MotionExplorer<T>::clear()
{
    Planner::clear();

    for (unsigned int k = 0; k < quotientSpaces_.size(); k++)
    {
        quotientSpaces_.at(k)->clear();
    }

    pdef_->clearSolutionPaths();
    for (unsigned int k = 0; k < pdefVec_.size(); k++)
    {
        pdefVec_.at(k)->clearSolutionPaths();
    }
    selectedPath_.clear();
}

template <class T>
void MotionExplorer<T>::setSelectedPath( std::vector<int> selectedPath){
  selectedPath_ = selectedPath;
  for(uint k = 0; k < selectedPath.size(); k++){
    //selected path implies path bias, which implies a sampling bias towards the
    //selected path
    quotientSpaces_.at(k)->selectedPath = selectedPath.at(k);
  }
  std::cout << "[SELECTION CHANGE] QuotientSpaces set to " << selectedPath << std::endl;
}

template <class T>
ob::PlannerStatus MotionExplorer<T>::solve(const ob::PlannerTerminationCondition &ptc)
{
  uint K = selectedPath_.size();
  if(K>=quotientSpaces_.size()){
    K = K-1;
  }
  og::QuotientGraphSparse *jQuotient = quotientSpaces_.at(K);

  uint ctr = 0;
  while (!ptc())
  {
    // std::cout << "Growing QuotientSpace " << jQuotient->getName() << std::endl;
    uint M = jQuotient->getNumberOfPaths();
    jQuotient->Grow();
    ctr++;
    uint Mg = jQuotient->getNumberOfPaths();
    //stop at topological phase shift
    if(Mg > M) return ob::PlannerStatus::APPROXIMATE_SOLUTION;
  }
  std::cout << "Grow QS for " << ctr << " iters." << std::endl;

  return ob::PlannerStatus::TIMEOUT;
}

template <class T>
void MotionExplorer<T>::setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_)
{
    if (siVec_.size() != pdef_.size())
    {
        OMPL_ERROR("Number of ProblemDefinitionPtr is %d but we have %d SpaceInformationPtr.", pdef_.size(), siVec_.size());
        exit(0);
    }
    pdefVec_ = pdef_;
    ob::Planner::setProblemDefinition(pdefVec_.back());
    for (unsigned int k = 0; k < pdefVec_.size(); k++)
    {
        quotientSpaces_.at(k)->setProblemDefinition(pdefVec_.at(k));
    }
}

template <class T>
void MotionExplorer<T>::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef)
{
    this->Planner::setProblemDefinition(pdef);
}

template <class T>
void MotionExplorer<T>::getPlannerData(ob::PlannerData &data) const
{
    unsigned int Nvertices = data.numVertices();
    if (Nvertices > 0)
    {
        std::cout << "cannot get planner data if plannerdata is already populated" << std::endl;
        std::cout << "PlannerData has " << Nvertices << " vertices." << std::endl;
        exit(0);
    }

    unsigned int K = quotientSpaces_.size();
    std::vector<uint> countVerticesPerQuotientSpace;

    for (unsigned int k = 0; k < K; k++)
    {
        og::Quotient *Qk = quotientSpaces_.at(k);
        static_cast<QuotientGraphSparse*>(Qk)->enumerateAllPaths();
        Qk->getPlannerData(data);
        // label all new vertices
        unsigned int ctr = 0;

        for (unsigned int vidx = Nvertices; vidx < data.numVertices(); vidx++)
        {
            PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated *>(&data.getVertex(vidx));
            v.SetLevel(k);
            v.SetMaxLevel(K);

            ob::State *s_lift = Qk->getSpaceInformation()->cloneState(v.getState());
            v.setQuotientState(s_lift);

            for (unsigned int m = k + 1; m < quotientSpaces_.size(); m++)
            {
                og::Quotient *Qm = quotientSpaces_.at(m);

                if (Qm->GetX1() != nullptr)
                {
                    ob::State *s_X1 = Qm->GetX1()->allocState();
                    ob::State *s_Q1 = Qm->getSpaceInformation()->allocState();
                    if (Qm->GetX1()->getStateSpace()->getType() == ob::STATE_SPACE_SO3)
                    {
                        static_cast<ob::SO3StateSpace::StateType *>(s_X1)->setIdentity();
                    }
                    if (Qm->GetX1()->getStateSpace()->getType() == ob::STATE_SPACE_SO2)
                    {
                        static_cast<ob::SO2StateSpace::StateType *>(s_X1)->setIdentity();
                    }
                    Qm->MergeStates(s_lift, s_X1, s_Q1);
                    s_lift = Qm->getSpaceInformation()->cloneState(s_Q1);

                    Qm->GetX1()->freeState(s_X1);
                    Qm->GetQ1()->freeState(s_Q1);
                }
            }
            v.setState(s_lift);
            ctr++;
        }
        countVerticesPerQuotientSpace.push_back(data.numVertices() - Nvertices);
        Nvertices = data.numVertices();

    }
    std::cout << "Created PlannerData with " << data.numVertices() << " vertices ";
    std::cout << "(";
    for(uint k = 0; k < countVerticesPerQuotientSpace.size(); k++){
       uint ck = countVerticesPerQuotientSpace.at(k);
       std::cout << ck << (k < countVerticesPerQuotientSpace.size()-1?", ":"");
    }
    std::cout << ")" << std::endl;
}

