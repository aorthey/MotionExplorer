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
  robot_ids.clear();
  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);
    robot_ids.push_back(ck->GetRobotIndex());
  }
}

void CSpaceOMPLMultiAgent::setNextLevelRobotPointers(std::vector<int> next_level_ids)
{
  for(uint k = 0; k < next_level_ids.size(); k++){
    int ik = next_level_ids.at(k);
    if(ik >=0){
      ptr_to_next_level_robot_ids.push_back(ik);
    }
  }
}

bool CSpaceOMPLMultiAgent::SatisfiesBounds(const ob::State *state)
{
  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);
    const ob::State *stateK = static_cast<const ob::CompoundState*>(state)->as<ob::State>(k);
    if(!ck->SatisfiesBounds(stateK)) return false;
  }
  return true;
}

bool CSpaceOMPLMultiAgent::isDynamic() const
{
  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);
    if(ck->isDynamic()) return true;
  }
  return false;
}

bool CSpaceOMPLMultiAgent::IsPlanar(){
  if(GetDimensionality()<=2) return true;

  //check if at least one robot is non-planar, i.e. freeFloating
  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);
    if(!ck->IsPlanar()) return false;
  }
  return true;
}

bool CSpaceOMPLMultiAgent::UpdateRobotConfig(Config &q)
{
  std::vector<Config> qks = splitConfig(q);
  for(uint k = 0; k < cspaces_.size(); k++){
    Config qk = qks.at(k);
    if(qk.size()<=0) continue;
    CSpaceOMPL *ck = cspaces_.at(k);
    Robot *robot = ck->GetRobotPtr();
    robot->UpdateConfig(qk);
    robot->UpdateGeometry();
  }
  return true;
}

bool CSpaceOMPLMultiAgent::isMultiAgent() const
{
  return true;
}

std::vector<int> CSpaceOMPLMultiAgent::GetRobotIdxs() const
{
  return robot_ids;
}
std::vector<int> CSpaceOMPLMultiAgent::GetProjectionIdxs() const
{
  return ptr_to_next_level_robot_ids;
}

void CSpaceOMPLMultiAgent::drawConfig(const Config &q, GLColor color, double scale){
  std::vector<Config> qks = splitConfig(q);
  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);
    if(ck->GetRobotIndex()>=0){
      ck->drawConfig(qks.at(k), color, scale);
    }
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
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "MultiAgentCspace" << std::endl;
  std::cout << std::string(80, '-') << std::endl;
  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);
    ck->print();
  }
  if(ptr_to_next_level_robot_ids.size()>0){
    std::cout << "Pointer To Next Level: " << std::endl;
    std::cout << ptr_to_next_level_robot_ids << std::endl;
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
  validity_checker = std::make_shared<OMPLValidityCheckerMultiAgent>(si, this, cspaces_);
  return validity_checker;
}

void CSpaceOMPLMultiAgent::ConfigToOMPLState(const Config &q, ob::State *qompl, int agent)
{
  ob::State *qomplAgent = static_cast<ob::CompoundState*>(qompl)->as<ob::State>(agent);
  cspaces_.at(agent)->ConfigToOMPLState(q, qomplAgent);
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
