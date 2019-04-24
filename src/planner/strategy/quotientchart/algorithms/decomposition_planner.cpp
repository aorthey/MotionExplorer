#include "common.h"

#include "decomposition_planner.h"
#include "elements/plannerdata_vertex_annotated.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <limits>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/transform_value_property_map.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

using namespace ompl::geometric;

DecompositionPlanner::DecompositionPlanner(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("DecompositionPlanner"+std::to_string(id));
}

DecompositionPlanner::~DecompositionPlanner(void)
{
}

void DecompositionPlanner::clear()
{
  BaseT::clear();
}

void DecompositionPlanner::setup(void)
{
  BaseT::setup();
}

void DecompositionPlanner::Grow(double t)
{
  if(firstRun){
    Init();
    firstRun=false;
  }

  ExtendGraphOneStep();
  if(hasSolution){
    Rewire();
  }
}

bool DecompositionPlanner::FoundNewComponent()
{
  if(chartNumberOfComponents<=0 && hasSolution){
    chartNumberOfComponents++;
    return true;
  }else{
    return false;
  }
}
