#include "quotient.h"
#include "planner/cspace/cspace.h"

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

using namespace ompl::geometric;
using namespace ompl::base;

uint Quotient::counter = 0;
void Quotient::resetCounter()
{
  Quotient::counter = 0;
}

Quotient::Quotient(const ob::SpaceInformationPtr &si, Quotient *previous_):
  ob::Planner(si,"QuotientPlanner"), previous(previous_), M1(si)
{
  const StateSpacePtr M1_space = M1->getStateSpace();
  M0_subspaces = 0;
  M1_subspaces = 0;

  id = counter++;
  std::cout << "--- Level " << id << " QuotientSpace" << std::endl;
  setName("Quotient"+std::to_string(id));
  if(previous == nullptr){
    std::cout << "M" << id <<" (dimension: " << M1_space->getDimension() << ") contains " << M1_subspaces << " subspaces of type " << M1_space->getName() << " " << M1_space->getType() << std::endl;
  }else{
    const StateSpacePtr M0_space = previous->getSpaceInformation()->getStateSpace();

    //create standalone CSpace M1/M0
    if(!M1_space->isCompound()){
      std::cout << "Cannot Split a non-compound configuration space" << std::endl;
      exit(0);
    }else{

      //get count of subspaces in M0
      if(!M0_space->isCompound()){
        M0_subspaces=1;
      }else{
        M0_subspaces = M0_space->as<ob::CompoundStateSpace>()->getSubspaceCount();
      }

      //get count of subspaces in M1
      if(!M1_space->isCompound()){
        std::cout << "M1 needs to be compound state" << std::endl;
        exit(0);
      }

      ob::CompoundStateSpace *M1_compound = M1_space->as<ob::CompoundStateSpace>();
      const std::vector<StateSpacePtr> M1_decomposed = M1_compound->getSubspaces();
      M1_subspaces = M1_decomposed.size();

      //sanity check it
      if(M0_space->getDimension() >= M1_space->getDimension()){
        std::cout << "Cannot reduce to a bigger cspace. We need dim(M0) < dim(M1)." << std::endl;
        std::cout << "M0 dimension: " << M0_space->getDimension() << std::endl;
        std::cout << "M1 dimension: " << M1_space->getDimension() << std::endl;
        exit(0);
      }

      if(M0_subspaces < M1_subspaces){
        //get C1
        C1_subspaces = M1_subspaces - M0_subspaces;

        if(C1_subspaces<=1){
          StateSpacePtr C1_space = M1_decomposed.at(M0_subspaces);
          C1 = std::make_shared<SpaceInformation>(C1_space);
        }else{
          StateSpacePtr C1_space = std::make_shared<ompl::base::CompoundStateSpace>();
          for(uint k = M0_subspaces; k < M0_subspaces+C1_subspaces; k++){
            C1_space->as<ob::CompoundStateSpace>()->addSubspace(M1_decomposed.at(k),1);
          }
          C1 = std::make_shared<SpaceInformation>(C1_space);
        }
      }else{
        StateSpacePtr M1_back = M1_decomposed.back();
        if((M1_back->getType() == base::STATE_SPACE_SO3) && (M0_space->getDimension() == 5)){
          StateSpacePtr C1_space = (std::make_shared<ob::SO2StateSpace>());
          C1 = std::make_shared<SpaceInformation>(C1_space);
          C1_subspaces = 1;
        }else{

          if(M0_space->getDimension() < M1_space->getDimension()){
            //try to fix it

            StateSpacePtr M1_1_space = M1_decomposed.at(0);
            if(M1_1_space->getDimension() == M0_space->getDimension()){
              C1 = std::make_shared<SpaceInformation>(M1_decomposed.at(1));
              C1_subspaces = 1;

            }else{
              std::cout << "M0 (dimension: " << M0_space->getDimension() << ") contains " << M0_subspaces << " subspaces of type " << M0_space->getName() << " " << M0_space->getType() << std::endl;
              std::cout << "M1 (dimension: " << M1->getStateDimension() << ") contains " << M1_subspaces  << " subspaces of type " << M1->getStateSpace()->getName() << " " << M1->getStateSpace()->getType() << std::endl;
              std::cout << "Subspaces do not match. We can only handle this in the case of S1xS1 subset SO(3)" << std::endl;
              exit(0);
            }


          }else{
            std::cout << "M0 (dimension: " << M0_space->getDimension() << ") contains " << M0_subspaces << " subspaces of type " << M0_space->getName() << " " << M0_space->getType() << std::endl;
            std::cout << "M1 (dimension: " << M1->getStateDimension() << ") contains " << M1_subspaces  << " subspaces of type " << M1->getStateSpace()->getName() << " " << M1->getStateSpace()->getType() << std::endl;
            std::cout << "Subspaces do not match. We can only handle this in the case of S1xS1 subset SO(3)" << std::endl;
            exit(0);
          }
        }

      }
      C1_sampler = C1->allocStateSampler();

    }
    std::cout << "M" << id-1 << " (dimension: " << M0_space->getDimension() << ") contains " << M0_subspaces << " subspaces of type " << M0_space->getName() << " " << M0_space->getType() << std::endl;
    std::cout << "M" << id << " (dimension: " << M1->getStateDimension() << ") contains " << M1_subspaces  << " subspaces of type " << M1->getStateSpace()->getName() << " " << M1->getStateSpace()->getType() << std::endl;
    std::cout << "C" << id << " (dimension: " << C1->getStateDimension() << ") contains " << C1_subspaces  << " subspaces of type " << C1->getStateSpace()->getName() << " " << C1->getStateSpace()->getType() << std::endl;

  }
}

void Quotient::mergeStates(const ob::State *qM0, const ob::State *qC1, ob::State *qM1)
{
  //input : qM0 \in M0, qC1 \in C1
  //output: qM1 = qM0 \circ qC1 \in M1
  const StateSpacePtr M1_space = M1->getStateSpace();
  const StateSpacePtr C1_space = C1->getStateSpace();
  const StateSpacePtr M0_space = previous->getSpaceInformation()->getStateSpace();

  ob::State **qM1_comps = qM1->as<CompoundState>()->components;
  ob::CompoundStateSpace *M1_compound = M1_space->as<ob::CompoundStateSpace>();
  const std::vector<StateSpacePtr> M1_subspaces = M1_compound->getSubspaces();
  StateSpacePtr M1_back = M1_subspaces.back();

  //si_->printState(qM0);
  //C1->printState(qC1);

  if((M1_back->getType() == base::STATE_SPACE_SO3) && (C1->getStateSpace()->getDimension() == 1)){
    //special case

    ob::State **sM0_comps = qM0->as<CompoundState>()->components;
    M1_subspaces[0]->copyState( qM1_comps[0], sM0_comps[0]);

    double r = sM0_comps[1]->as<ob::SO2StateSpace::StateType>()->value;
    double p = sM0_comps[2]->as<ob::SO2StateSpace::StateType>()->value;
    double y = qC1->as<ob::SO2StateSpace::StateType>()->value;

    ob::SE3StateSpace::StateType *qM1SE3 = qM1->as<ob::SE3StateSpace::StateType>();
    ob::SO3StateSpace::StateType *qM1SO3 = &qM1SE3->rotation();

    CSpaceOMPL::OMPLSO3StateSpaceFromEulerXYZ(r, p, y, qM1_comps[1]->as<ob::SO3StateSpace::StateType>() );

  }else{
    //M0 SE3 compound, M1 compound SE3+RN, C1 RN, basically, we cannot just copy
    //M0 SE3 to M1 SE3, we need to iterate through compounds
    //std::cout << std::string(80, '-') << std::endl;
    //std::cout << "qM0" << std::endl;
    //std::cout << std::string(80, '-') << std::endl;
    //M0_space->printState(qM0);
    //std::cout << std::string(80, '-') << std::endl;
    //std::cout << "qC1" << std::endl;
    //std::cout << std::string(80, '-') << std::endl;
    //C1_space->printState(qC1);
    if(M1_subspaces.at(0)->isCompound()){
      ob::CompoundStateSpace *M1_comp_comp = M1_subspaces.at(0)->as<ob::CompoundStateSpace>();
      const std::vector<StateSpacePtr> M1_1_subspaces = M1_comp_comp->getSubspaces();
      //std::cout << "subspaces: " << M1_1_subspaces.size() << std::endl;
      ob::State **qM1_comps_comps = qM1_comps[0]->as<CompoundState>()->components;
  //ob::State **qM1_comps = qM1->as<CompoundState>()->components;
      ob::State **sM0_comps = qM0->as<CompoundState>()->components;
      for(uint k = 0; k < M0_subspaces; k++){
        M1_1_subspaces[k]->copyState( qM1_comps_comps[k], sM0_comps[k]);
      }

      M1_subspaces[1]->copyState( qM1_comps[1], qC1);
    }else{
      if(M0_subspaces > 1){
        ob::State **sM0_comps = qM0->as<CompoundState>()->components;
        for(uint k = 0; k < M0_subspaces; k++){
          M1_subspaces[k]->copyState( qM1_comps[k], sM0_comps[k]);
        }
      }else{
        M1_subspaces[0]->copyState( qM1_comps[0], qM0);
      }
      if(C1_subspaces > 1){
        ob::State **sC1_comps = qC1->as<CompoundState>()->components;
        for(uint k = M0_subspaces; k < M1_subspaces.size(); k++){
          M1_subspaces[k]->copyState( qM1_comps[k], sC1_comps[k-M0_subspaces]);
        }
      }else{
        M1_subspaces[M0_subspaces]->copyState( qM1_comps[M0_subspaces], qC1);
      }
    }
    //std::cout << std::string(80, '-') << std::endl;
    //std::cout << "qM1" << std::endl;
    //std::cout << std::string(80, '-') << std::endl;
    //M1_space->printState(qM1);
    //if(M1_subspaces.at(0)->isCompound()){
    //  exit(0);
    //}
  }
}
bool Quotient::Sample(ob::State *q_random)
{
  if(previous == nullptr){
    return sampler_->sample(q_random);
  }else{
    //Adjusted sampling function: Sampling in G0 x C1

    ob::SpaceInformationPtr M0 = previous->getSpaceInformation();
    base::State *s_C1 = C1->allocState();
    base::State *s_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_C1);
    previous->SampleGraph(s_M0);
    mergeStates(s_M0, s_C1, q_random);

    return M1->isValid(q_random);
  }
}
void Quotient::ExtractC1Subspace( ob::State* q, ob::State* qC1 ) const
{
  ob::State **q_comps = q->as<CompoundState>()->components;
  ob::CompoundStateSpace *M1_compound = M1->getStateSpace()->as<ob::CompoundStateSpace>();
  const std::vector<StateSpacePtr> subspaces = M1_compound->getSubspaces();

  if(C1_subspaces > 1){
    ob::State **sC1_comps = qC1->as<CompoundState>()->components;
    for(uint k = M0_subspaces; k < subspaces.size(); k++){
      subspaces[k]->copyState( sC1_comps[k-M0_subspaces], q_comps[k]);
    }
  }else{

    StateSpacePtr M1_back = subspaces.back();
    if((M1_back->getType() == base::STATE_SPACE_SO3) && (C1->getStateSpace()->getDimension() == 1)){

      ob::SO3StateSpace::StateType *SO3 = &q->as<ob::SE3StateSpace::StateType>()->rotation();

      std::vector<double> EulerXYZ = CSpaceOMPL::EulerXYZFromOMPLSO3StateSpace( SO3);

      qC1->as<ob::SO2StateSpace::StateType>()->value = EulerXYZ.at(2);

    }else{
      subspaces[M0_subspaces]->copyState( qC1, q_comps[M0_subspaces]);
    }
  }
}

void Quotient::ExtractM0Subspace( ob::State* q, ob::State* qM0 ) const
{
  ob::State **q_comps = q->as<CompoundState>()->components;
  ob::CompoundStateSpace *M1_compound = M1->getStateSpace()->as<ob::CompoundStateSpace>();
  const std::vector<StateSpacePtr> subspaces = M1_compound->getSubspaces();

  if(M0_subspaces > 1){
    ob::State **sM0_comps = qM0->as<CompoundState>()->components;

    StateSpacePtr M1_back = subspaces.back();
    if((M1_back->getType() == base::STATE_SPACE_SO3) && (C1->getStateSpace()->getDimension() == 1)){


      ob::SO3StateSpace::StateType *SO3 = &q->as<ob::SE3StateSpace::StateType>()->rotation();

      std::vector<double> EulerXYZ = CSpaceOMPL::EulerXYZFromOMPLSO3StateSpace( SO3);

      //qC1->as<ob::SO2StateSpace::StateType>()->value = EulerXYZ.at(2);

      subspaces[0]->copyState( sM0_comps[0], q_comps[0]);
      sM0_comps[1]->as<ob::SO2StateSpace::StateType>()->value = EulerXYZ.at(0);
      sM0_comps[2]->as<ob::SO2StateSpace::StateType>()->value = EulerXYZ.at(1);

      //std::cout << "special case" << std::endl;

    }else{
      for(uint k = 0; k < M0_subspaces; k++){
        subspaces[k]->copyState( sM0_comps[k], q_comps[k]);
      }
    }
  }else{
    subspaces[0]->copyState( qM0, q_comps[0]);
  }
}
bool Quotient::SampleGraph(ob::State *q_random)
{
  std::cout << "NYI" << std::endl;
  exit(0);
  return false;
}
