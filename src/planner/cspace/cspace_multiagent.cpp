#include "planner/cspace/cspace_multiagent.h"
#include "planner/cspace/validitychecker/validity_checker_multiagent.h"

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

const oc::StatePropagatorPtr CSpaceOMPLMultiAgent::StatePropagatorPtr(oc::SpaceInformationPtr si)
{
  OMPL_ERROR("MultiAgentCspace has no StatePropagatorPtr");
  throw "No StatePropagatorPtr.";
}

void CSpaceOMPLMultiAgent::initSpace()
{
  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);
    ck->initSpace();
    space = space + ck->SpacePtr();
    Nklampts.push_back( ck->GetKlamptDimensionality() );
    Nompls.push_back( ck->GetDimensionality() );
  }
  Nompl = std::accumulate(Nompls.begin(), Nompls.end(), 0);
  Nklampt = std::accumulate(Nklampts.begin(), Nklampts.end(), 0);
}

uint CSpaceOMPLMultiAgent::GetKlamptDimensionality() const{
  std::cout << "Klampt DIM: " << Nklampt << std::endl;
  return Nklampt;
}

void CSpaceOMPLMultiAgent::print(std::ostream& out) const
{
  for(uint k = 0; k < cspaces_.size(); k++){
    CSpaceOMPL *ck = cspaces_.at(k);
    ck->print();
  }
}

void CSpaceOMPLMultiAgent::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
  std::cout << q << std::endl;
  OMPL_ERROR("NYI");
  throw "NYI";
}
Config CSpaceOMPLMultiAgent::OMPLStateToConfig(const ob::State *qompl)
{
  OMPL_ERROR("NYI");
  throw "NYI";
}

const ob::StateValidityCheckerPtr CSpaceOMPLMultiAgent::StateValidityCheckerPtr(ob::SpaceInformationPtr si)
{
  validity_checker = std::make_shared<OMPLValidityCheckerMultiAgent>(si, cspaces_);
  return validity_checker;
}
