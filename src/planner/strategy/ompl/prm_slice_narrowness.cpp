#include "prm_slice_narrowness.h"
#include "planner/cspace/cspace.h"

#include <ompl/datastructures/PDF.h>
#include <boost/foreach.hpp>
#include <boost/graph/graphviz.hpp>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>

#include <boost/graph/one_bit_color_map.hpp>
#include <boost/graph/stoer_wagner_min_cut.hpp>
#include <boost/property_map/property_map.hpp>

using namespace og;
using namespace ob;
typedef og::PRMBasic::Edge Edge;
#define foreach BOOST_FOREACH

//******************************************************************************
//******************************************************************************
PRMSliceNarrow::PRMSliceNarrow(const ob::SpaceInformationPtr &si, PRMSliceNarrow *previous_):
  PRMSlice(si, previous_)
{
  setName("PRMSliceNarrow"+to_string(id));
}

PRMSliceNarrow::~PRMSliceNarrow(){
  std::cout << "delete PRMSliceNarrow" << to_string(id) << std::endl;
}

ompl::PDF<Edge> PRMSliceNarrow::GetEdgePDF()
{
  PDF<Edge> pdf;
  foreach (Edge e, boost::edges(g_))
  {
    ob::Cost weight = get(boost::edge_weight_t(), g_, e).getCost();
    pdf.add(e, weight.value());
  }
  return pdf;
}

//******************************************************************************
//******************************************************************************
PRMSliceNarrowEdgeDegree::PRMSliceNarrowEdgeDegree(const ob::SpaceInformationPtr &si, PRMSliceNarrow *previous_):
  PRMSliceNarrow(si, previous_)
{
  setName("PRMSliceNarrow(EdgeDegree)"+to_string(id));
}
ompl::PDF<Edge> PRMSliceNarrowEdgeDegree::GetEdgePDF(){
  PDF<Edge> pdf;
  foreach (Edge e, boost::edges(g_))
  {
    const Vertex v1 = boost::source(e, g_);
    const Vertex v2 = boost::target(e, g_);
    uint d1 = boost::degree(v1, g_);
    uint d2 = boost::degree(v2, g_);

    uint de = d1*d2;
    if(de>0){
      ob::Cost weight = get(boost::edge_weight_t(), g_, e).getCost();
      double d = weight.value()*1.0/((double)pow(de,16));
      pdf.add(e, d);
    }else{
      pdf.add(e, 0);
    }
  }
  return pdf;
}

//******************************************************************************
//******************************************************************************
PRMSliceNarrowMinCut::PRMSliceNarrowMinCut(const ob::SpaceInformationPtr &si, PRMSliceNarrow *previous_):
  PRMSliceNarrow(si, previous_)
{
  setName("PRMSliceNarrow(MinCut)"+to_string(id));
}
ompl::PDF<Edge> PRMSliceNarrowMinCut::GetEdgePDF(){

  BOOST_AUTO(parities, boost::make_one_bit_color_map(num_vertices(g_), get(boost::vertex_index, g_)));
  
  auto wmap = make_transform_value_property_map([](EdgeProperty& e) { return e.getCost().value(); }, get(boost::edge_weight_t(), g_));

  int w = boost::stoer_wagner_min_cut(g_, wmap, boost::parity_map(parities));

  PDF<Edge> pdf;
  foreach (Edge e, boost::edges(g_))
  {
    const Vertex v1 = boost::source(e, g_);
    const Vertex v2 = boost::target(e, g_);
    bool p1 = get(parities,v1);
    bool p2 = get(parities,v2);
    if(p1==p2){
      pdf.add(e,0);
    }else{
      ob::Cost weight = get(boost::edge_weight_t(), g_, e).getCost();
      pdf.add(e, weight.value());
    }
  }

  return pdf;
}


