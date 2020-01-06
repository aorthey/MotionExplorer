#include "gui/colors.h"
#include "common.h"
#include "planner/cspace/cspace_multiagent.h"
#include "planner/cspace/validitychecker/validity_checker_multiagent.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <boost/foreach.hpp>
#include <numeric>

CSpaceOMPLMultiAgent::CSpaceOMPLMultiAgent(std::vector<CSpaceOMPL*> cspaces):
  CSpaceOMPL(cspaces.front()->GetWorldPtr(), cspaces.front()->GetRobotIndex()), cspaces_(cspaces)
{
}

bool CSpaceOMPLMultiAgent::isDynamic() const
{
  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);
    if(ck->isDynamic()) return true;
  }
  return false;
}

bool CSpaceOMPLMultiAgent::isMultiAgent() const
{
  return true;
}

std::vector<int> CSpaceOMPLMultiAgent::GetRobotIdxs() const
{
  std::vector<int> idxs;
  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);
    idxs.push_back(ck->GetRobotIndex());
  }
  return idxs;
}

void CSpaceOMPLMultiAgent::drawConfig(const Config &q, GLColor color, double scale){
  std::vector<Config> qks = splitConfig(q);
  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);
    ck->drawConfig(qks.at(k), color, scale);
  }
}

const oc::StatePropagatorPtr CSpaceOMPLMultiAgent::StatePropagatorPtr(oc::SpaceInformationPtr si)
{
  OMPL_ERROR("MultiAgentCspace has no StatePropagatorPtr");
  throw "No StatePropagatorPtr.";
}

void CSpaceOMPLMultiAgent::initSpace()
{
  space = std::make_shared<ob::CompoundStateSpace>();

  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);
    ck->initSpace();
    // space = space + ck->SpacePtr();

    static_pointer_cast<ob::CompoundStateSpace>(space)->addSubspace(ck->SpacePtr(), 1);

    Nklampts.push_back( ck->GetKlamptDimensionality() );
    Nompls.push_back( ck->GetDimensionality() );
  }

  Nompl = std::accumulate(Nompls.begin(), Nompls.end(), 0);
  Nklampt = std::accumulate(Nklampts.begin(), Nklampts.end(), 0);

  if(!space->isCompound()){
    OMPL_ERROR("Not compound state space");
    throw "Not compound state space";
  }

}

uint CSpaceOMPLMultiAgent::GetKlamptDimensionality() const{
  return Nklampt;
}
std::vector<int> CSpaceOMPLMultiAgent::GetKlamptDimensionalities() const{
  return Nklampts;
}

void CSpaceOMPLMultiAgent::print(std::ostream& out) const
{
  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);
    ck->print();
  }
}

std::vector<Config> CSpaceOMPLMultiAgent::splitConfig(const Config &q)
{
  int ctr = 0;
  std::vector<Config> qks;
  for(uint k = 0; k < Nklampts.size(); k++){
    int Nk = Nklampts.at(k);
    Config qk; qk.resize(Nk);
    for(int j = 0; j < Nk; j++){
      qk[j] = q[j+ctr];
    }
    qks.push_back(qk);
    ctr+=Nk;
  }
  return qks;
}

Vector3 CSpaceOMPLMultiAgent::getXYZ(const ob::State *qompl)
{
  return getXYZ(qompl, GetRobotIdxs().front());
}

Vector3 CSpaceOMPLMultiAgent::getXYZ(const ob::State *qompl, int ridx)
{
  std::vector<int> ridxs = GetRobotIdxs();
  for(uint k = 0; k < ridxs.size(); k++){
    if(ridx == ridxs.at(k)){
      const ob::State *qomplAgent = static_cast<const ob::CompoundState*>(qompl)->as<ob::State>(k);
      return cspaces_.at(k)->getXYZ(qomplAgent);
    }
  }
  OMPL_ERROR("Could not find robot index %d.", ridx);
  throw "NoIndex";
}

const ob::StateValidityCheckerPtr CSpaceOMPLMultiAgent::StateValidityCheckerPtr(ob::SpaceInformationPtr si)
{
  validity_checker = std::make_shared<OMPLValidityCheckerMultiAgent>(si, cspaces_);
  return validity_checker;
}



void CSpaceOMPLMultiAgent::ConfigToOMPLState(const Config &q, ob::State *qompl, int agent)
{
  //computes the first agent-elements (i.e. [0,agent[). [0,2[ = [0,1]

  ob::State *qomplAgent = static_cast<ob::CompoundState*>(qompl)->as<ob::State>(agent);

  cspaces_.at(agent)->ConfigToOMPLState(q, qomplAgent);

  // cspaces_.at(agent)->SpaceInformationPtr()->printState(qomplAgent);

}

void CSpaceOMPLMultiAgent::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  int ctr = 0;
  std::vector<Config> qks = splitConfig(q);
  for(uint k = 0; k < Nklampts.size(); k++){
    int Nk = Nklampts.at(k);
    Config qk; qk.resize(Nk);
    for(int j = 0; j < Nk; j++){
      qk[j] = q[j+ctr];
    }
    qks.push_back(qk);
    ctr+=Nk;
  }

  for(uint k = 0; k < cspaces_.size(); k++){
    SpaceInformationPtr()->printState(qompl);
    ConfigToOMPLState(qks.at(k), qompl, k);
  }
}


Config CSpaceOMPLMultiAgent::OMPLStateToConfig(const ob::State *qompl, int agent)
{
  const ob::State *qomplAgent = static_cast<const ob::CompoundState*>(qompl)->as<ob::State>(agent);
  Config qk = cspaces_.at(agent)->OMPLStateToConfig(qomplAgent);
  return qk;
}
Config CSpaceOMPLMultiAgent::OMPLStateToConfig(const ob::State *qompl)
{
  int ctr = 0;
  Config q; q.resize(Nklampt);
  for(uint k = 0; k < cspaces_.size(); k++){
    Config qk = OMPLStateToConfig(qompl, k);
    for(int j = 0; j < qk.size(); j++){
      q[j+ctr] = qk[j];
    }
    ctr += qk.size();
  }
  return q;
}
