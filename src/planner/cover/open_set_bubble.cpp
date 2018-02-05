#include "open_set_bubble.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace cover;

OpenSetBubble::OpenSetBubble(ob::SpaceInformationPtr si_, ob::State *s, double dist_robot_env_):
  OpenSet(si_,s,dist_robot_env)
{
  ob::StateSpacePtr space = si->getStateSpace();
  if( space->getType() == ob::STATE_SPACE_REAL_VECTOR ){
    type = RN;
  }else if(space->getType() == ob::STATE_SPACE_SE2){
    type = SE2;
  }else if(space->getType() == ob::STATE_SPACE_SE3){
    type = SE3;
  }else{
    ob::CompoundStateSpace *space_compound = space->as<ob::CompoundStateSpace>();
    const std::vector<ob::StateSpacePtr> space_decomposed = space_compound->getSubspaces();
    if(   space_decomposed.size() == 2
        && space_decomposed.at(0)->getType() == ob::STATE_SPACE_SE3
        && space_decomposed.at(1)->getType() == ob::STATE_SPACE_REAL_VECTOR){
      type = SE3RN;
    }else{
      std::cout << "wrong space type: " << space->getType() << std::endl;
    }
  }
}


double OpenSetBubble::Distance(const ob::State *s_lhs, const ob::State *s_rhs){
  if(type == RN){
    uint dim = si->getStateDimension();
    if(dim != 3){
      OMPL_ERROR("cannot handle %d dimensions",dim);
      exit(0);
    }
    const double *RN_lhs = s_lhs->as<ob::RealVectorStateSpace::StateType>()->values;
    const double *RN_rhs = s_rhs->as<ob::RealVectorStateSpace::StateType>()->values;

    double dist = 0;
    for (unsigned int i = 0; i < 3; ++i)
    {
      dist += fabs( (*RN_lhs++) - (*RN_rhs++));
    }
    //std::cout << std::string(80, '-') << std::endl;
    //si->printState(s_lhs);
    //si->printState(s_rhs);
    //std::cout << "dist: " << dist << " radius open set: " << dist_robot_env << std::endl;
    //std::cout << std::string(80, '-') << std::endl;
    return dist;
  }else{
    //ob::SE3StateSpace::StateType *q_lhs = s_lhs->as<ob::SE3StateSpace::StateType>();
    //ob::SO3StateSpace::StateType *q_lhs_rot = &q_lhs->rotation();

    //double x_lhs = q_lhs->getX();
    //double y_lhs = q_lhs->getY();
    //double z_lhs = q_lhs->getZ();
    //std::cout << x_lhs << std::endl;
    std::cout << "NYI" << std::endl;
    exit(0);
  }
}


