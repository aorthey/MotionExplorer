//#include "rrt_quotient_cover.h"
//#include "planner/validitychecker/validity_checker_ompl.h"
//#include "elements/plannerdata_vertex_annotated.h"
//#include <ompl/base/goals/GoalSampleableRegion.h>
//#include <ompl/tools/config/SelfConfig.h>
//#include <ompl/base/PlannerData.h>
//
//using namespace ompl::geometric;
//using namespace ompl::base;
//using namespace og;
//
//RRTQuotientCover::RRTQuotientCover(const base::SpaceInformationPtr &si, Quotient *previous_) : 
//  RRTQuotient(si, previous_)
//{
//  setName("RRTQuotientCover");
//}
//
//RRTQuotient::GrowState RRTQuotient::growTree(TreeData &tree, TreeGrowingInfo &tgi,
//                                                                             Configuration *rconfiguration)
//{
//    /* find closest state in the tree */
//    Configuration *nconfiguration = tree->nearest(rconfiguration);
//
//    /* assume we can reach the state we go towards */
//    bool reach = true;
//
//    /* find state to add */
//    base::State *dstate = rconfiguration->state;
//    double d = si_->distance(nconfiguration->state, rconfiguration->state);
//    if (d > maxDistance_)
//    {
//        si_->getStateSpace()->interpolate(nconfiguration->state, rconfiguration->state, maxDistance_ / d, tgi.xstate);
//        dstate = tgi.xstate;
//        reach = false;
//    }
//    // if we are in the start tree, we just check the configuration like we normally do;
//    // if we are in the goal tree, we need to check the configuration in reverse, but checkMotion() assumes the first state it
//    // receives as argument is valid,
//    // so we check that one first
//    bool validConfiguration = tgi.start ?
//                           si_->checkMotion(nconfiguration->state, dstate) :
//                           si_->getStateValidityChecker()->isValid(dstate) && si_->checkMotion(dstate, nconfiguration->state);
//
//    if (validConfiguration)
//    {
//      auto *configuration = new Configuration(si_);
//      si_->copyState(configuration->state, dstate);
//      configuration->parent = nconfiguration;
//      configuration->root = nconfiguration->root;
//      configuration->parent_edge_weight = distanceFunction(configuration, configuration->parent);
//
//      auto checkerPtr = static_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());
//      configuration->openNeighborhoodDistance = checkerPtr->Distance(configuration->state);
//
//      if(checkerPtr->IsSufficient(configuration->state))
//      {
//        configuration->isSufficient = true;
//      }
//
//      tgi.xconfiguration = configuration;
//      tree->add(configuration);
//      return reach ? REACHED : ADVANCED;
//    }
//    else
//      return TRAPPED;
//}
//
//
//void RRTQuotient::Grow(double t)
//{
//  if(isTreeConnected){
//
//  }else{
//    RRTQuotient::Grow(t);
//    if(isTreeConnected){
//      //merge start and goal tree by reversing goal tree.
//    }
//  }
//}
//
//bool RRTQuotient::SampleGraph(ob::State *q_random_graph)
//{
//
//  PDF<Configuration*> pdf = GetConfigurationPDF();
//
//  Configuration *q = pdf.sample(rng_.uniform01());
//  double t = rng_.uniform01();
//  //if(t<0.05){
//  //  double tc = rng_.uniform01();
//  //  const ob::State *from = connectionPoint_.first;
//  //  const ob::State *to = connectionPoint_.second;
//  //  M1->getStateSpace()->interpolate(from, to, tc, q_random_graph);
//  //}else{
//  //  const ob::State *from = q->state;
//  //  const ob::State *to = q->parent->state;
//  //  M1->getStateSpace()->interpolate(from, to, t, q_random_graph);
//  //}
//  const ob::State *from = q->state;
//  const ob::State *to = q->parent->state;
//  M1->getStateSpace()->interpolate(from, to, t, q_random_graph);
//
//  return true;
//}
