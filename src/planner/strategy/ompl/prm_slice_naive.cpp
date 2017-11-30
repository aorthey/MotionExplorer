#include "prm_slice_naive.h"
#include <ompl/datastructures/PDF.h>
#include <boost/foreach.hpp>

using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH
//
//  Visualization of CSpace Decomposition
//
// [    ][    ]
// [    ][    ]
// [    ][ M0 ]
// [ M1 ][____]
// [    ][    ]
// [    ][ C1 ]
// [    ][    ]
//
// whereby 
// M1 = M1
// M0 = previous->getspaceinformation()
// C1 = C1
//
// Standard PRM is sampling in M1
// PRMSlice is sampling in G0 x C0, whereby G0 is the roadmap on M0
//
//
//Multilevel M0 \subspace M1 \subspace M2
//
// [    ][    ][    ]
// [    ][    ][ M0 ]
// [    ][    ][    ]
// [    ][ M1 ][____]
// [    ][    ]
// [ M2 ][    ]
// [    ][    ]
// [    ][____]
// [    ]
// [    ]
// [____]
//
// [    ][    ][    ]
// [    ][    ][ C0 ]
// [    ][    ][    ]
// [    ][ M1 ][____]
// [    ][    ][    ]
// [ M2 ][    ][ C1 ]
// [    ][    ][    ]
// [    ][____][____]
// [    ][    ][    ]
// [    ][ C2 ][ C2 ]
// [____][____][____]

PRMSliceNaive::PRMSliceNaive(const ob::SpaceInformationPtr &si, PRMSliceNaive *previous_ ):
  og::PRMSlice(si), previous(previous_), M1(si)
{
  const StateSpacePtr M1_space = M1->getStateSpace();
  M0_subspaces = 0;
  M1_subspaces = 0;

  if(previous == nullptr){
    std::cout << "Level 0 SliceSpace" << std::endl;
    //C1 = std::make_shared<SpaceInformation>(M1);
  }else{
    std::cout << "Level 1 SliceSpace" << std::endl;
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

      std::cout << "M0 contains " << M0_subspaces << " subspaces." << std::endl;
      std::cout << "M1 contains " << M1_subspaces << " subspaces." << std::endl;

      //sanity check it
      if(M0_subspaces >= M1_subspaces){
        std::cout << "Cannot reduce to a bigger cspace. We need M0 < M1." << std::endl;
        exit(0);
      }

      //get C1
      C1_subspaces = M1_subspaces - M0_subspaces;

      StateSpacePtr C1_space;
      if(C1_subspaces<=1){
        C1_space = M1_decomposed.at(M0_subspaces);
      }else{
        for(uint k = M0_subspaces; k < M0_subspaces+C1_subspaces; k++){
          C1_space->as<ob::CompoundStateSpace>()->addSubspace(M1_decomposed.at(k),1);
        }
      }
      C1 = std::make_shared<SpaceInformation>(C1_space);

      //get count of subspaces in C1
      uint C1_subspaces = 0;
      if(!C1_space->isCompound()){
        C1_subspaces=1;
      }else{
        C1_subspaces = C1_space->as<ob::CompoundStateSpace>()->getSubspaceCount();
      }
      std::cout << "C1 contains " << C1_subspaces << " subspaces." << std::endl;
    }
    if (!C1_sampler){
      C1_sampler = C1->allocStateSampler();
    }

  }
  //const StateValidityCheckerPtr checker = si->getStateValidityChecker();
  //C1->setStateValidityChecker(checker);

}

PRMSliceNaive::~PRMSliceNaive(){
}


bool PRMSliceNaive::Sample(ob::State *workState){
  if(previous == nullptr){
    //without any underlying CSpace, the behavior is equal to PRM
    return sampler_->sample(workState);
  }else{
    //Adjusted sampling function: Sampling in G0 x C1

    ob::SpaceInformationPtr M0 = previous->getSpaceInformation();
    base::State *s_C1 = C1->allocState();
    base::State *s_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_C1);
    previous->SampleGraph(s_M0);

    //workState is element of M1, how do we get values for sub components!?

    std::cout << "M0 : "; M0->printState(s_M0, std::cout);
    std::cout << "C1 : "; C1->printState(s_C1, std::cout);

    State **workState_comps = workState->as<CompoundState>()->components;
    ob::CompoundStateSpace *M1_compound = M1->getStateSpace()->as<ob::CompoundStateSpace>();
    const std::vector<StateSpacePtr> subspaces = M1_compound->getSubspaces();

    if(M0_subspaces > 1){
      State **sM0_comps = s_M0->as<CompoundState>()->components;
      for(uint k = 0; k < M0_subspaces; k++){
        subspaces[k]->copyState( workState_comps[k], sM0_comps[k]);
      }
    }else{
      subspaces[0]->copyState( workState_comps[0], s_M0);
    }
    if(C1_subspaces > 1){
      State **sC1_comps = s_C1->as<CompoundState>()->components;
      for(uint k = M0_subspaces; k < subspaces.size(); k++){
        subspaces[k]->copyState( workState_comps[k], sC1_comps[k-M0_subspaces]);
      }
    }else{
      subspaces[M0_subspaces]->copyState( workState_comps[M0_subspaces], s_C1);
    }

    std::cout << "M1 : "; M1->printState(workState, std::cout);


    //workState = s_M1 = s_C1+s_M0
    //check if the compound state is valid!
    return true;
  }
}

bool PRMSliceNaive::SampleGraph(ob::State *workState){
  PDF<Edge> pdf;
  foreach (Edge e, boost::edges(g_))
  {
    ob::Cost weight = get(boost::edge_weight_t(), g_, e);
    pdf.add(e, weight.value());
  }
  if(pdf.empty()){
    std::cout << "cannot sample empty(?) graph" << std::endl;
    exit(0);
  }
  Edge e = pdf.sample(rng_.uniform01());
  double t = rng_.uniform01();
  //std::cout << "sampled edge " << e << " and t=" << t << std::endl;

  const Vertex v1 = boost::source(e, g_);
  const Vertex v2 = boost::target(e, g_);
  const ob::State *from = stateProperty_[v1];
  const ob::State *to = stateProperty_[v2];

  M1->getStateSpace()->interpolate(from, to, t, workState);

  //this is necessarily feasible
  return true;

}
