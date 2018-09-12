#include "common.h"
#include "qng.h"
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

QNG::QNG(const base::SpaceInformationPtr &si, Quotient *parent ): BaseT(si, parent)
{
  setName("QNG"+std::to_string(id));
}

QNG::~QNG(void)
{
}

void QNG::Grow(double t)
{
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(t) );
  //#########################################################################
  //Do not grow the cover if it is saturated, i.e. it cannot be expanded anymore
  // --- we have successfully computed Q1_{free}, the free space of Q1
  //#########################################################################
  if(saturated) return;

  //#########################################################################
  //Sample a configuration different from the current cover
  //#########################################################################
  Configuration *q_random = SampleValid(ptc);
  if(q_random == nullptr) return;

  AddConfigurationToCover(q_random);

  std::vector<Vertex> path = GetCoverPath(v_start, get(indexToVertex, q_random->index));
  std::vector<uint> ipath;
  for(uint k = 0; k < path.size(); k++){
    ipath.push_back( get(vertexToIndex, path.at(k)) );
  }
  std::cout << "path start " << graph[v_start]->index << " to " << q_random->index << ":" << ipath << std::endl;
}

