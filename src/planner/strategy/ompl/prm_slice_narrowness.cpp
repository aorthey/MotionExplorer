#include "prm_slice_narrowness.h"
#include "planner/cspace/cspace.h"

#include <ompl/datastructures/PDF.h>
#include <boost/foreach.hpp>
#include <boost/graph/graphviz.hpp>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

using namespace og;
using namespace ob;
#define foreach BOOST_FOREACH

PRMSliceNarrow::PRMSliceNarrow(const ob::SpaceInformationPtr &si, PRMSliceNarrow *previous_ ):
  PRMSlice(si, previous_)
{
  setName("PRMSliceNarrow"+to_string(id));
}

PRMSliceNarrow::~PRMSliceNarrow(){
  std::cout << "delete PRMSLiceNarrow" << to_string(id) << std::endl;
}

bool PRMSliceNarrow::SampleGraph(ob::State *workState){
  PDF<Edge> pdf;
  Vertex v1o,v2o;
  foreach (Edge e, boost::edges(g_))
  {
    ob::Cost weight = get(boost::edge_weight_t(), g_, e).getCost();
    const Vertex v1 = boost::source(e, g_);
    const Vertex v2 = boost::target(e, g_);
    uint d1 = boost::degree(v1, g_);
    uint d2 = boost::degree(v2, g_);

    uint de = d1*d2;

    if(de>0){
      double d = weight.value()*1.0/((double)pow(de,16));
      pdf.add(e, d);
    }else{
      //pdf.add(e, weight.value());
      pdf.add(e, 0);
    }

    //pdf.add(e, weight.value());



    // if(DEBUG){
    //   if(v1o==v1 && v2o==v2){
    //     std::cout << std::string(80, '-') << std::endl;
    //     std::cout << "Double edge (num edges: " << num_edges(g_) << ")" << std::endl;
    //     std::cout << "Edge: (" << v1 << "," << v2 << ") weight: "<<weight.value() << std::endl;
    //     std::cout << std::string(80, '-') << std::endl;
    //     uint ctr = 0;
    //     foreach (Edge e, boost::edges(g_))
    //     {
    //       ctr++;
    //     }
    //     std::cout << "edges counted: " << ctr << std::endl;
    //     exit(0);
    //   }
    //   if(v1o==v2 && v2o==v1){
    //     std::cout << std::string(80, '-') << std::endl;
    //     std::cout << "Double edge (num edges: " << num_edges(g_) << ")" << std::endl;
    //     std::cout << "Edge: (" << v1 << "," << v2 << ") weight: "<<weight.value() << std::endl;
    //     std::cout << std::string(80, '-') << std::endl;
    //     uint ctr = 0;
    //     foreach (Edge e, boost::edges(g_))
    //     {
    //       ctr++;
    //     }
    //     std::cout << "edges counted: " << ctr << std::endl;
    //     exit(0);
    //   }

    //   v1o = v1;
    //   v2o = v2;
    // }
  }
  if(pdf.empty()){
    std::cout << "cannot sample empty(?) graph" << std::endl;
    exit(0);
  }

  Edge e = pdf.sample(rng_.uniform01());
  double t = rng_.uniform01();

  const Vertex v1 = boost::source(e, g_);
  const Vertex v2 = boost::target(e, g_);
  const ob::State *from = stateProperty_[v1];
  const ob::State *to = stateProperty_[v2];

  M1->getStateSpace()->interpolate(from, to, t, workState);

  lastSourceVertexSampled = v1;
  lastTargetVertexSampled = v2;
  lastTSampled = t;

  isSampled = true;

  return true;

}
