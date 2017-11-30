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
// M1 = si_current
// M0 = previous->getspaceinformation()
// C1 = si_standalone
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
  og::PRMSlice(si), previous(previous_), si_current(si)
{
  const StateSpacePtr M1 = si->getStateSpace();

  if(previous == nullptr){
    std::cout << "Level 0 SliceSpace" << std::endl;
    si_standalone = std::make_shared<SpaceInformation>(M1);
  }else{
    std::cout << "Level 1 SliceSpace" << std::endl;
    const StateSpacePtr M0 = previous->getSpaceInformation()->getStateSpace();

    //create standalone CSpace M1/M0
    if(!M1->isCompound()){
      std::cout << "Cannot Split a non-compound configuration space" << std::endl;
      exit(0);
    }else{

      //get count of subspaces in M0
      uint M0_subspaces = 0;
      if(!M0->isCompound()){
        M0_subspaces=1;
      }else{
        M0_subspaces = M0->as<ob::CompoundStateSpace>()->getSubspaceCount();
      }

      //get count of subspaces in M1
      ob::CompoundStateSpace *M1_compound = M1->as<ob::CompoundStateSpace>();
      const std::vector<StateSpacePtr> M1_decomposed = M1_compound->getSubspaces();
      uint M1_subspaces = M1_decomposed.size();

      std::cout << "M0 contains " << M0_subspaces << " subspaces." << std::endl;
      std::cout << "M1 contains " << M1_subspaces << " subspaces." << std::endl;

      //sanity check it
      if(M0_subspaces >= M1_subspaces){
        std::cout << "Cannot reduce to a bigger cspace. We need M0 < M1." << std::endl;
        exit(0);
      }

      uint Ms_subspaces = M1_subspaces - M0_subspaces;

      StateSpacePtr C1;
      if(Ms_subspaces<=1){
        C1 = M1_decomposed.at(M0_subspaces);
      }else{
        for(uint k = M0_subspaces; k < M0_subspaces+Ms_subspaces; k++){
          C1->as<ob::CompoundStateSpace>()->addSubspace(M1_decomposed.at(k),1);
        }
      }
      si_standalone = std::make_shared<SpaceInformation>(C1);

      //get count of subspaces in C1
      uint C1_subspaces = 0;
      if(!C1->isCompound()){
        C1_subspaces=1;
      }else{
        C1_subspaces = C1->as<ob::CompoundStateSpace>()->getSubspaceCount();
      }
      std::cout << "C1 contains " << C1_subspaces << " subspaces." << std::endl;
      //M0->printSettings(std::cout);
      //M1->printSettings(std::cout);
      //si_standalone->printSettings(std::cout);
    }

  }
  //const StateValidityCheckerPtr checker = si->getStateValidityChecker();
  //si_standalone->setStateValidityChecker(checker);

  if (!C1_sampler){
    C1_sampler = si_standalone->allocStateSampler();
  }
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
    base::State *s_C1 = si_standalone->allocState();
    base::State *s_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_C1);
    previous->SampleGraph(s_M0);

    //workState = s_M1 = s_C1+s_M0
    //check if the compound state is valid!
    return true;
  }
}

bool PRMSliceNaive::SampleGraph(ob::State *workState){
  //g_
  //  std::cout << "  edges : " << boost::num_edges(graph) << std::endl;
  //boost::property_map<Graph, boost::edge_weight_t>::const_type
  //  foreach (const Edge e, boost::edges(graph))
  //  {
  //      const Vertex v1 = boost::source(e, graph);
  //      const Vertex v2 = boost::target(e, graph);
  //      data.addEdge(ob::PlannerDataVertex(stateProperty_[v1]), ob::PlannerDataVertex(stateProperty_[v2]));
  //      data.addEdge(ob::PlannerDataVertex(stateProperty_[v2]), ob::PlannerDataVertex(stateProperty_[v1]));
  //      data.tagState(stateProperty_[v1], const_cast<PRMPlain *>(this)->disjointSets_.find_set(v1));
  //      data.tagState(stateProperty_[v2], const_cast<PRMPlain *>(this)->disjointSets_.find_set(v2));
  //  }
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
  std::cout << "sampled edge " << e << " and t=" << t << std::endl;
  exit(0);

}
