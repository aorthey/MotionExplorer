#include "prm_quotient_narrowness.h"
#include "planner/cspace/cspace.h"
#include "planner/validitychecker/validity_checker_ompl.h"

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
PRMQuotientNarrow::PRMQuotientNarrow(const ob::SpaceInformationPtr &si, Quotient *previous_):
  PRMQuotient(si, previous_)
{
  setName("PRMQuotientNarrow"+to_string(id));
}

PRMQuotientNarrow::~PRMQuotientNarrow(){
  std::cout << "delete PRMQuotientNarrow" << to_string(id) << std::endl;
}

ompl::PDF<Edge> PRMQuotientNarrow::GetEdgePDF()
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
PRMQuotientNarrowEdgeDegree::PRMQuotientNarrowEdgeDegree(const ob::SpaceInformationPtr &si, Quotient *previous_):
  PRMQuotientNarrow(si, previous_)
{
  setName("PRMQuotientNarrow(EdgeDegree)"+to_string(id));
}
ompl::PDF<Edge> PRMQuotientNarrowEdgeDegree::GetEdgePDF(){
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
PRMQuotientNarrowMinCut::PRMQuotientNarrowMinCut(const ob::SpaceInformationPtr &si, Quotient *previous_):
  PRMQuotientNarrow(si, previous_)
{
  setName("PRMQuotientNarrow(MinCut)"+to_string(id));
}
ompl::PDF<Edge> PRMQuotientNarrowMinCut::GetEdgePDF(){

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

//******************************************************************************
//******************************************************************************
PRMQuotientNarrowDistance::PRMQuotientNarrowDistance(const ob::SpaceInformationPtr &si, Quotient *previous_):
  PRMQuotientNarrow(si, previous_)
{
  setName("PRMQuotientNarrowDistance"+to_string(id));
}

ompl::PDF<Edge> PRMQuotientNarrowDistance::GetEdgePDF()
{
  auto checkerPtr = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());

  PDF<Edge> pdf;

  foreach (Edge e, boost::edges(g_))
  {
    //ob::Cost weight = get(boost::edge_weight_t(), g_, e).getCost();

    const Vertex v1 = boost::source(e, g_);
    const Vertex v2 = boost::target(e, g_);

    double d1 = checkerPtr->Distance(stateProperty_[v1]);
    double d2 = checkerPtr->Distance(stateProperty_[v2]);
    double d = 1.0/min(d1,d2);
    pdf.add(e, d);
  }
  return pdf;
}

double PRMQuotientNarrowDistance::GetSamplingDensity()
{
  return (double)num_vertices(g_);
}
