#include "quotient.h"
#include "planner/cspace/cspace.h"

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

using namespace ompl::geometric;
using namespace ompl::base;


Quotient::Quotient(const ob::SpaceInformationPtr &si, Quotient *parent_):
  ob::Planner(si,"QuotientSpace"), Q1(si), Q0(si), parent(parent_)
{
  id = counter++;

  if(parent!=nullptr) parent->SetChild(this); //need to be able to traverse down the tree

  const StateSpacePtr Q1_space = Q1->getStateSpace();

  if(verbose>0) std::cout << "--- QuotientSpace" << id << std::endl;

  if(parent == nullptr){
    if(verbose>0) std::cout << "Q1 dimension : " << Q1_space->getDimension() << " measure: " << Q1_space->getMeasure() << std::endl;
    type = ATOMIC_RN;
  }else{
    Q0 = parent->getSpaceInformation();
    const StateSpacePtr Q0_space = parent->getSpaceInformation()->getStateSpace();

    //X1 = Q1 / Q0
    const StateSpacePtr X1_space = ComputeQuotientSpace(Q1_space, Q0_space);

    X1 = std::make_shared<SpaceInformation>(X1_space);
    X1_sampler = X1->allocStateSampler();

    if(Q0_space->getDimension()+X1_space->getDimension() != Q1_space->getDimension()){
      std::cout << "quotient of state spaces got dimensions wrong." << std::endl;
      exit(0);
    }
    if(verbose>0) std::cout << "Q0 dimension : " << Q0_space->getDimension() << " measure: " << Q0_space->getMeasure() << std::endl;
    if(verbose>0) std::cout << "X1 dimension : " << X1_space->getDimension() << " measure: " << X1_space->getMeasure() << std::endl;
    if(verbose>0) std::cout << "Q1 dimension : " << Q1_space->getDimension() << " measure: " << Q1_space->getMeasure() << std::endl;
    if((Q0_space->getMeasure()<=0) ||
       (X1_space->getMeasure()<=0) ||
       (Q1_space->getMeasure()<=0)){
      std::cout << "zero-measure quotient space detected. abort." << std::endl;
      exit(0);
    }
    if (!X1_sampler){
      X1_sampler = X1->allocStateSampler();
    }
  }

  if (!Q1_valid_sampler){
    Q1_valid_sampler = Q1->allocValidStateSampler();
  }
  if (!Q1_sampler){
    Q1_sampler = Q1->allocStateSampler();
  }

  checker = dynamic_pointer_cast<OMPLValidityCheckerNecessarySufficient>(si->getStateValidityChecker());
  if(!checker){
    OMPL_INFORM("Validity Checker only uses inner robot.");
    checkOuterRobot = false;
    checker = dynamic_pointer_cast<OMPLValidityChecker>(si_->getStateValidityChecker());
    if(!checker){
      OMPL_ERROR("Validity Checker cannot compute distances.");
      exit(0);
    }
  }else{
    checkOuterRobot = true;
  }
  // ob::State *state = Q1->allocState();
  // std::cout << "TEST NEIGHBORHOOD" << std::endl;
  // checker->Distance(state);
  // Q1->freeState(state);

}

bool Quotient::IsOuterRobotFeasible(ob::State *state)
{
  if(checkOuterRobot){
    return checker->IsSufficientFeasible(state);
  }else{
    return false;
  }
}
double Quotient::DistanceOuterRobotToObstacle(ob::State *state)
{
  if(checkOuterRobot){
    return checker->SufficientDistance(state);
  }else{
    return 0.0;
  }
}
double Quotient::DistanceInnerRobotToObstacle(ob::State *state)
{
  return checker->Distance(state);
}

double Quotient::DistanceRobotToObstacle(ob::State *state)
{
  if(checkOuterRobot){
    if(IsOuterRobotFeasible(state)){
      return DistanceOuterRobotToObstacle(state);
    }else{
      return DistanceInnerRobotToObstacle(state);
    }
  }else{
    return DistanceInnerRobotToObstacle(state);
  }
}

ob::PlannerStatus Quotient::solve(const ob::PlannerTerminationCondition &ptc)
{
  OMPL_ERROR("A Quotient-Space cannot be solved alone. Use class MultiQuotient to solve Quotient-Spaces.");
  exit(1);
}

uint Quotient::counter = 0;
void Quotient::resetCounter()
{
  Quotient::counter = 0;
}

void Quotient::clear()
{
  Planner::clear();
  hasSolution = false;
  Quotient::counter = 0;
  //totalNumberOfSamples = 0;
  //graphLength = 0;
  if(parent==nullptr) X1_sampler.reset();
}

const StateSpacePtr Quotient::ComputeQuotientSpace(const StateSpacePtr Q1, const StateSpacePtr Q0)
{
    //STATE_SPACE_UNKNOWN = 0,
    //STATE_SPACE_REAL_VECTOR = 1,
    //STATE_SPACE_SO2 = 2,
    //STATE_SPACE_SO3 = 3,
    //STATE_SPACE_SE2 = 4,
    //STATE_SPACE_SE3 = 5,
    //STATE_SPACE_TIME = 6,
    //STATE_SPACE_DISCRETE = 7,

    //Cases we can handle:
    // ---- non-compound:
    // (1) Q1 Rn       , Q0 Rm       , 0<m<n   => X1 = R(n-m)
    // ---- compound:
    // (2) Q1 SE2      , Q0 R2                 => X1 = SO2
    // (3) Q1 SE3      , Q0 R3                 => X1 = SO3
    // (4) Q1 SE3xRn   , Q0 SE3                => X1 = Rn
    // (5) Q1 SE3xRn   , Q0 R3                 => X1 = SO3xRn
    // (6) Q1 SE3xRn   , Q0 SE3xRm   , 0<m<n   => X1 = R(n-m)
    //
    // Cases not yet implemented
    ///// Q1 SE3      , Q0 R3xSO2xSO2         =>X1 = SO2
    ///// Q1 R3xS1xS1 , Q0 R3                 =>X1 = SO2xSO2

    if(!Q1->isCompound()){
      ///##############################################################################/
      //------------------ non-compound cases:
      ///##############################################################################/
      //
      //------------------ (1) Q1 = Rn, Q0 = Rm, 0<m<n, X1 = R(n-m)
      if( Q1->getType() == base::STATE_SPACE_REAL_VECTOR ){
        uint n = Q1->getDimension();
        if( Q0->getType() == base::STATE_SPACE_REAL_VECTOR ){
          uint m = Q0->getDimension();
          if(n>m && m>0){
            type = RN_RM;
          }else{
            std::cout << "not allowed: we need n>m>0, but have " << n << ">" << m << ">0" << std::endl;
            exit(0);
          }
        }else{
          std::cout << "Q1 is R^"<<n <<" but state " << Q0->getType() << " is not handled." << std::endl;
          exit(0);
        }
      }else{
        std::cout << "Q1 is non-compound state, but state " << Q1->getType() << " is not handled." << std::endl;
        exit(0);
      }
    }else{
      ///##############################################################################/
      //------------------ compound cases:
      ///##############################################################################/
      //
      //------------------ (2) Q1 = SE2, Q0 = R2, X1 = SO2
      ///##############################################################################/
      if( Q1->getType() == base::STATE_SPACE_SE2 ){
        if( Q0->getType() == base::STATE_SPACE_REAL_VECTOR){
          if( Q0->getDimension() == 2){
            type = SE2_R2;
          }else{
            std::cout << "Q1 is SE2 but state " << Q0->getType() << " is of dimension " << Q0->getDimension() << std::endl;
            exit(0);
          }
        }else{
          std::cout << "Q1 is SE2 but state " << Q0->getType() << " is not handled." << std::endl;
          exit(0);
        }
      }
      //------------------ (3) Q1 = SE3, Q0 = R3, X1 = SO3
      ///##############################################################################/
      else if( Q1->getType() == base::STATE_SPACE_SE3 ){
        if( Q0->getType() == base::STATE_SPACE_REAL_VECTOR){
          if( Q0->getDimension() == 3){
            type = SE3_R3;
          }else{
            std::cout << "Q1 is SE3 but state " << Q0->getType() << " is of dimension " << Q0->getDimension() << std::endl;
            exit(0);
          }
        }else{
          std::cout << "Q1 is SE3 but state " << Q0->getType() << " is not handled." << std::endl;
          exit(0);
        }
      }
      //------------------ Q1 = SE3xRN
      ///##############################################################################/
      else{
        ob::CompoundStateSpace *Q1_compound = Q1->as<ob::CompoundStateSpace>();
        const std::vector<StateSpacePtr> Q1_decomposed = Q1_compound->getSubspaces();
        uint Q1_subspaces = Q1_decomposed.size();
        if(Q1_subspaces == 2){
          if(Q1_decomposed.at(0)->getType() == base::STATE_SPACE_SE3
              && Q1_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR){

            uint n = Q1_decomposed.at(1)->getDimension();
            if( Q0->getType() == base::STATE_SPACE_SE3 ){
            //------------------ (4) Q1 = SE3xRn, Q0 = SE3, X1 = Rn
            ///##############################################################################/
              type = SE3RN_SE3;
            }else if(Q0->getType() == base::STATE_SPACE_REAL_VECTOR){
            //------------------ (5) Q1 = SE3xRn, Q0 = R3, X1 = SO3xRN
            ///##############################################################################/
              type = SE3RN_R3;
            }else{
            //------------------ (6) Q1 = SE3xRn, Q0 = SE3xRm, X1 = R(n-m)
            ///##############################################################################/
              ob::CompoundStateSpace *Q0_compound = Q0->as<ob::CompoundStateSpace>();
              const std::vector<StateSpacePtr> Q0_decomposed = Q0_compound->getSubspaces();
              uint Q0_subspaces = Q0_decomposed.size();
              if(Q0_subspaces==2){
                if(Q1_decomposed.at(0)->getType() == base::STATE_SPACE_SE3
                    && Q1_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR){
                  uint m = Q0_decomposed.at(1)->getDimension();
                  if(m<n && m>0){
                    type = SE3RN_SE3RM;
                  }else{
                    std::cout << "not allowed: we need n>m>0, but have " << n << ">" << m << ">0" << std::endl;
                    exit(0);
                  }
                }
              }else{
                std::cout << "State compound with " << Q0_subspaces << " not handled."<< std::endl;
                exit(0);
              }
            }
          }else{
            std::cout << "State compound " <<  Q1_decomposed.at(0)->getType() << " and " <<  Q1_decomposed.at(1)->getType() << " not recognized."<< std::endl;
            exit(0);
          }
        }else{
          std::cout << "Q1 has " << Q1_subspaces << " but we only support 2." << std::endl;
          exit(0);
        }
      }

    }

    StateSpacePtr X1;
    Q1_dimension = Q1->getDimension();
    Q0_dimension = Q0->getDimension();

    switch (type) {
      case RN_RM:
        {
          uint N = Q1_dimension - Q0_dimension;
          X1 = std::make_shared<ob::RealVectorStateSpace>(N);
          X1_dimension = N;

          RealVectorBounds Q1_bounds = static_pointer_cast<ob::RealVectorStateSpace>(Q1)->getBounds();
          std::vector<double> low; low.resize(N);
          std::vector<double> high; high.resize(N);
          RealVectorBounds X1_bounds(N);
          for(uint k = 0; k < N; k++){
            X1_bounds.setLow(k, Q1_bounds.low.at(k+Q0_dimension));
            X1_bounds.setHigh(k, Q1_bounds.high.at(k+Q0_dimension));
          }
          static_pointer_cast<ob::RealVectorStateSpace>(X1)->setBounds(X1_bounds);

          break;
        }
      case SE2_R2:
        {
          X1 = std::make_shared<ob::SO2StateSpace>();
          break;
        }
      case SE3_R3:
        {
          X1 = std::make_shared<ob::SO3StateSpace>();
          break;
        }
      case SE3RN_SE3:
        {
          ob::CompoundStateSpace *Q1_compound = Q1->as<ob::CompoundStateSpace>();
          const std::vector<StateSpacePtr> Q1_decomposed = Q1_compound->getSubspaces();

          X1_dimension = Q1_decomposed.at(1)->getDimension();

          X1 = std::make_shared<ob::RealVectorStateSpace>(X1_dimension);
          static_pointer_cast<ob::RealVectorStateSpace>(X1)->setBounds( static_pointer_cast<ob::RealVectorStateSpace>(Q1_decomposed.at(1))->getBounds() );

          break;
        }
      case SE3RN_R3:
        {
          ob::CompoundStateSpace *Q1_compound = Q1->as<ob::CompoundStateSpace>();
          const std::vector<StateSpacePtr> Q1_decomposed = Q1_compound->getSubspaces();
          const std::vector<StateSpacePtr> Q1_SE3_decomposed = Q1_decomposed.at(0)->as<ob::CompoundStateSpace>()->getSubspaces();

          //const ob::SE3StateSpace *Q1_SE3 = Q1_SE3_decomposed.at(0)->as<ob::SE3StateSpace>();
          //const ob::SO3StateSpace *Q1_SO3 = Q1_SE3_decomposed.at(1)->as<ob::SO3StateSpace>();
          const ob::RealVectorStateSpace *Q1_RN = Q1_decomposed.at(1)->as<ob::RealVectorStateSpace>();
          uint N = Q1_RN->getDimension();

          ob::StateSpacePtr SO3(new ob::SO3StateSpace());
          ob::StateSpacePtr RN(new ob::RealVectorStateSpace(N));
          RN->as<ob::RealVectorStateSpace>()->setBounds( Q1_RN->getBounds() );

          X1 = SO3 + RN;
          X1_dimension = 3+N;
          break;
        }
      case SE3RN_SE3RM:
        {
          ob::CompoundStateSpace *Q1_compound = Q1->as<ob::CompoundStateSpace>();
          const std::vector<StateSpacePtr> Q1_decomposed = Q1_compound->getSubspaces();
          ob::CompoundStateSpace *Q0_compound = Q0->as<ob::CompoundStateSpace>();
          const std::vector<StateSpacePtr> Q0_decomposed = Q0_compound->getSubspaces();

          uint N = Q1_decomposed.at(1)->getDimension();
          uint M = Q0_decomposed.at(1)->getDimension();
          X1_dimension = N-M;
          X1 = std::make_shared<ob::RealVectorStateSpace>(X1_dimension);

          RealVectorBounds Q1_bounds = static_pointer_cast<ob::RealVectorStateSpace>(Q1_decomposed.at(1))->getBounds();
          std::vector<double> low; low.resize(X1_dimension);
          std::vector<double> high; high.resize(X1_dimension);
          RealVectorBounds X1_bounds(X1_dimension);
          for(uint k = 0; k < X1_dimension; k++){
            X1_bounds.setLow(k, Q1_bounds.low.at(k+M));
            X1_bounds.setHigh(k, Q1_bounds.high.at(k+M));
          }
          static_pointer_cast<ob::RealVectorStateSpace>(X1)->setBounds(X1_bounds);
          break;
        }
      default:
        {
          std::cout << "unknown type: " << type << std::endl;
          exit(0);
        }

    }
    return X1;

}

void Quotient::MergeStates(const ob::State *qQ0, const ob::State *qX1, ob::State *qQ1) const
{
  ////input : qQ0 \in Q0, qX1 \in X1
  ////output: qQ1 = qQ0 \circ qX1 \in Q1
  const StateSpacePtr Q1_space = Q1->getStateSpace();
  const StateSpacePtr X1_space = X1->getStateSpace();
  const StateSpacePtr Q0_space = parent->getSpaceInformation()->getStateSpace();

  switch (type) {
    case RN_RM:
      {
        ob::RealVectorStateSpace::StateType *sQ1 = qQ1->as<RealVectorStateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *sQ0 = qQ0->as<RealVectorStateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *sX1 = qX1->as<RealVectorStateSpace::StateType>();

        for(uint k = 0; k < Q0_dimension; k++){
          sQ1->values[k] = sQ0->values[k];
        }
        for(uint k = Q0_dimension; k < Q1_dimension; k++){
          sQ1->values[k] = sX1->values[k-Q0_dimension];
        }
        break;
      }
    case SE2_R2:
      {
        ob::SE2StateSpace::StateType *sQ1 = qQ1->as<SE2StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *sQ0 = qQ0->as<RealVectorStateSpace::StateType>();
        const ob::SO2StateSpace::StateType *sX1 = qX1->as<SO2StateSpace::StateType>();

        sQ1->setXY( sQ0->values[0], sQ0->values[1] );
        sQ1->setYaw( sX1->value );

        break;
      }
    case SE3_R3:
      {
        ob::SE3StateSpace::StateType *sQ1 = qQ1->as<SE3StateSpace::StateType>();
        ob::SO3StateSpace::StateType *sQ1_rotation = &sQ1->rotation();

        const ob::RealVectorStateSpace::StateType *sQ0 = qQ0->as<RealVectorStateSpace::StateType>();
        const ob::SO3StateSpace::StateType *sX1 = qX1->as<SO3StateSpace::StateType>();

        sQ1->setXYZ( sQ0->values[0], sQ0->values[1], sQ0->values[2]);

        sQ1_rotation->x = sX1->x;
        sQ1_rotation->y = sX1->y;
        sQ1_rotation->z = sX1->z;
        sQ1_rotation->w = sX1->w;

        break;
      }
    case SE3RN_R3:
      {
        ob::SE3StateSpace::StateType *sQ1_SE3 = qQ1->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        ob::SO3StateSpace::StateType *sQ1_SO3 = &sQ1_SE3->rotation();
        ob::RealVectorStateSpace::StateType *sQ1_RN = qQ1->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        const ob::RealVectorStateSpace::StateType *sQ0 = qQ0->as<RealVectorStateSpace::StateType>();
        const ob::SO3StateSpace::StateType *sX1_SO3 = qX1->as<ob::CompoundState>()->as<ob::SO3StateSpace::StateType>(0);
        const ob::RealVectorStateSpace::StateType *sX1_RN = qX1->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        sQ1_SE3->setXYZ( sQ0->values[0], sQ0->values[1], sQ0->values[2]);
        sQ1_SO3->x = sX1_SO3->x;
        sQ1_SO3->y = sX1_SO3->y;
        sQ1_SO3->z = sX1_SO3->z;
        sQ1_SO3->w = sX1_SO3->w;

        for(uint k = 0; k < X1_dimension-3; k++){
          sQ1_RN->values[k] = sX1_RN->values[k];
        }

        break;
      }
    case SE3RN_SE3:
      {
        ob::SE3StateSpace::StateType *sQ1_SE3 = qQ1->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        ob::SO3StateSpace::StateType *sQ1_SE3_rotation = &sQ1_SE3->rotation();
        ob::RealVectorStateSpace::StateType *sQ1_RN = qQ1->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        const ob::SE3StateSpace::StateType *sQ0 = qQ0->as<SE3StateSpace::StateType>();
        const ob::SO3StateSpace::StateType *sQ0_rotation = &sQ0->rotation();
        const ob::RealVectorStateSpace::StateType *sX1 = qX1->as<RealVectorStateSpace::StateType>();

        sQ1_SE3->setXYZ( sQ0->getX(), sQ0->getY(), sQ0->getZ());
        sQ1_SE3_rotation->x = sQ0_rotation->x;
        sQ1_SE3_rotation->y = sQ0_rotation->y;
        sQ1_SE3_rotation->z = sQ0_rotation->z;
        sQ1_SE3_rotation->w = sQ0_rotation->w;

        for(uint k = 0; k < X1_dimension; k++){
          sQ1_RN->values[k] = sX1->values[k];
        }

        break;
      }
    case SE3RN_SE3RM:{
        ob::SE3StateSpace::StateType *sQ1_SE3 = qQ1->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        ob::SO3StateSpace::StateType *sQ1_SE3_rotation = &sQ1_SE3->rotation();
        ob::RealVectorStateSpace::StateType *sQ1_RN = qQ1->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        const ob::SE3StateSpace::StateType *sQ0_SE3 = qQ0->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        const ob::SO3StateSpace::StateType *sQ0_SE3_rotation = &sQ0_SE3->rotation();
        const ob::RealVectorStateSpace::StateType *sQ0_RM = qQ0->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        const ob::RealVectorStateSpace::StateType *sX1 = qX1->as<RealVectorStateSpace::StateType>();

        sQ1_SE3->setXYZ( sQ0_SE3->getX(), sQ0_SE3->getY(), sQ0_SE3->getZ());
        sQ1_SE3_rotation->x = sQ0_SE3_rotation->x;
        sQ1_SE3_rotation->y = sQ0_SE3_rotation->y;
        sQ1_SE3_rotation->z = sQ0_SE3_rotation->z;
        sQ1_SE3_rotation->w = sQ0_SE3_rotation->w;

        //[X Y Z YAW PITCH ROLL] [1...M-1][M...N-1]
        //SE3                    RN
        uint M = Q1_dimension-X1_dimension-6;
        uint N = X1_dimension;

        for(uint k = 0; k < M; k++){
          sQ1_RN->values[k] = sQ0_RM->values[k];
        }
        for(uint k = M; k < M+N; k++){
          sQ1_RN->values[k] = sX1->values[k-M];
        }
        break;
      }
    default:
      {
        std::cout << "cannot merge states" << std::endl;
        std::cout << "type : " << type << " not implemented." << std::endl;
        exit(0);
      }

  }
}
void Quotient::ExtractX1Subspace( const ob::State* q, ob::State* qX1 ) const
{
  switch(type){
    case RN_RM:
      {
        const ob::RealVectorStateSpace::StateType *sQ1 = q->as<RealVectorStateSpace::StateType>();
        ob::RealVectorStateSpace::StateType *sX1 = qX1->as<RealVectorStateSpace::StateType>();

        for(uint k = Q0_dimension; k<Q1_dimension; k++){
          sX1->values[k-Q0_dimension] = sQ1->values[k];
        }
        break;
      }
    case SE2_R2:
      {
        const ob::SE2StateSpace::StateType *sQ1 = q->as<SE2StateSpace::StateType>();
        ob::SO2StateSpace::StateType *sX1 = qX1->as<SO2StateSpace::StateType>();
        sX1->value = sQ1->getYaw();
        break;
      }
    case SE3_R3:
      {
        const ob::SE3StateSpace::StateType *sQ1 = q->as<SE3StateSpace::StateType>();
        const ob::SO3StateSpace::StateType *sQ1_SO3 = &sQ1->rotation();

        ob::SO3StateSpace::StateType *sX1_SO3 = qX1->as<SO3StateSpace::StateType>();

        sX1_SO3->x = sQ1_SO3->x;
        sX1_SO3->y = sQ1_SO3->y;
        sX1_SO3->z = sQ1_SO3->z;
        sX1_SO3->w = sQ1_SO3->w;

        break;
      }
    case SE3RN_R3:
      {
        const ob::SE3StateSpace::StateType *sQ1_SE3 = q->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        const ob::SO3StateSpace::StateType *sQ1_SO3 = &sQ1_SE3->rotation();
        const ob::RealVectorStateSpace::StateType *sQ1_RN = q->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        ob::SO3StateSpace::StateType *sX1_SO3 = qX1->as<ob::CompoundState>()->as<SO3StateSpace::StateType>(0);
        ob::RealVectorStateSpace::StateType *sX1_RN = qX1->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        sX1_SO3->x = sQ1_SO3->x;
        sX1_SO3->y = sQ1_SO3->y;
        sX1_SO3->z = sQ1_SO3->z;
        sX1_SO3->w = sQ1_SO3->w;
        for(uint k = 0; k < X1_dimension-3; k++){
          sX1_RN->values[k] = sQ1_RN->values[k];
        }

        break;
      }
    case SE3RN_SE3:
      {
        const ob::RealVectorStateSpace::StateType *sQ1_RN = q->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);
        ob::RealVectorStateSpace::StateType *sX1 = qX1->as<RealVectorStateSpace::StateType>();

        for(uint k = 0; k < X1_dimension; k++){
          sX1->values[k] = sQ1_RN->values[k];
        }

        break;
      }
    case SE3RN_SE3RM:{
        const ob::RealVectorStateSpace::StateType *sQ1_RN = q->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        const ob::RealVectorStateSpace::StateType *sX1 = qX1->as<RealVectorStateSpace::StateType>();

        uint N = Q1_dimension - X1_dimension - 6;
        for(uint k = N; k < Q1_dimension-6; k++){
          sX1->values[k-N] = sQ1_RN->values[k];
        }
        break;
      }
    default:
      {
        std::cout << "cannot merge states" << std::endl;
        exit(0);
      }

  }
}

void Quotient::ExtractQ0Subspace( const ob::State* q, ob::State* qQ0 ) const
{

  switch(type){
    case RN_RM:
      {
        const ob::RealVectorStateSpace::StateType *sQ1 = q->as<RealVectorStateSpace::StateType>();
        ob::RealVectorStateSpace::StateType *sQ0 = qQ0->as<RealVectorStateSpace::StateType>();

        for(uint k = 0; k<Q0_dimension; k++){
          sQ0->values[k] = sQ1->values[k];
        }
        break;
      }
    case SE2_R2:
      {
        const ob::SE2StateSpace::StateType *sQ1 = q->as<SE2StateSpace::StateType>();
        ob::RealVectorStateSpace::StateType *sQ0 = qQ0->as<RealVectorStateSpace::StateType>();
        sQ0->values[0] = sQ1->getX();
        sQ0->values[1] = sQ1->getY();
        break;
      }
    case SE3_R3:
      {
        const ob::SE3StateSpace::StateType *sQ1 = q->as<SE3StateSpace::StateType>();
        ob::RealVectorStateSpace::StateType *sQ0 = qQ0->as<RealVectorStateSpace::StateType>();

        sQ0->values[0] = sQ1->getX();
        sQ0->values[1] = sQ1->getY();
        sQ0->values[2] = sQ1->getZ();

        break;
      }
    case SE3RN_R3:
      {
        const ob::SE3StateSpace::StateType *sQ1_SE3 = q->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        //const ob::SO3StateSpace::StateType *sQ1_SO3 = &sQ1_SE3->rotation();
        //const ob::RealVectorStateSpace::StateType *sQ1_RN = q->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        ob::RealVectorStateSpace::StateType *sQ0 = qQ0->as<RealVectorStateSpace::StateType>();

        sQ0->values[0] = sQ1_SE3->getX();
        sQ0->values[1] = sQ1_SE3->getY();
        sQ0->values[2] = sQ1_SE3->getZ();

        break;
      }
    case SE3RN_SE3:
      {
        const ob::SE3StateSpace::StateType *sQ1_SE3 = q->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        const ob::SO3StateSpace::StateType *sQ1_SE3_rotation = &sQ1_SE3->rotation();

        ob::SE3StateSpace::StateType *sQ0 = qQ0->as<SE3StateSpace::StateType>();
        ob::SO3StateSpace::StateType *sQ0_rotation = &sQ0->rotation();

        sQ0->setXYZ( sQ1_SE3->getX(), sQ1_SE3->getY(), sQ1_SE3->getZ());
        sQ0_rotation->x = sQ1_SE3_rotation->x;
        sQ0_rotation->y = sQ1_SE3_rotation->y;
        sQ0_rotation->z = sQ1_SE3_rotation->z;
        sQ0_rotation->w = sQ1_SE3_rotation->w;

        break;
      }
    case SE3RN_SE3RM:
      {
        const ob::SE3StateSpace::StateType *sQ1_SE3 = q->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        const ob::SO3StateSpace::StateType *sQ1_SE3_rotation = &sQ1_SE3->rotation();
        const ob::RealVectorStateSpace::StateType *sQ1_RN = q->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        ob::SE3StateSpace::StateType *sQ0_SE3 = qQ0->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        ob::SO3StateSpace::StateType *sQ0_rotation = &sQ0_SE3->rotation();
        ob::RealVectorStateSpace::StateType *sQ0_RN = qQ0->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        sQ0_SE3->setXYZ( sQ1_SE3->getX(), sQ1_SE3->getY(), sQ1_SE3->getZ());
        sQ0_rotation->x = sQ1_SE3_rotation->x;
        sQ0_rotation->y = sQ1_SE3_rotation->y;
        sQ0_rotation->z = sQ1_SE3_rotation->z;
        sQ0_rotation->w = sQ1_SE3_rotation->w;

        for(uint k = 0; k < Q0_dimension-6; k++){
          sQ0_RN->values[k] = sQ1_RN->values[k];
        }
        break;
      }
    default:
      {
        std::cout << "cannot merge states" << std::endl;
        exit(0);
      }

  }
}

const ob::SpaceInformationPtr& Quotient::GetX1() const
{
  return X1;
}
const ob::SpaceInformationPtr& Quotient::GetQ1() const
{
  return Q1;
}
const ob::SpaceInformationPtr& Quotient::GetQ0() const
{
  return Q0;
}

bool Quotient::HasSolution()
{
  if(!hasSolution){
    ob::PathPtr path;
    hasSolution = GetSolution(path);
  }
  return hasSolution;
}
Quotient* Quotient::GetParent() const
{
  return parent;
}
Quotient* Quotient::GetChild() const
{
  return child;
}
void Quotient::SetChild(Quotient *child_)
{
  child = child_;
}
void Quotient::SetParent(Quotient *parent_)
{
  parent = parent_;
}
uint Quotient::GetLevel() const
{
  return level;
}
void Quotient::SetLevel(uint level_)
{
  level = level_;
}
Quotient::QuotientSpaceType Quotient::GetType() const
{
  return type;
}

bool Quotient::Sample(ob::State *q_random)
{
  if(parent == nullptr){
    return Q1_valid_sampler->sample(q_random);
  }else{
    //Adjusted sampling function: Sampling in G0 x X1
    ob::SpaceInformationPtr Q0 = parent->GetQ1(); //Q0 is Q1 of parent!
    base::State *s_X1 = X1->allocState();
    base::State *s_Q0 = Q0->allocState();

    X1_sampler->sampleUniform(s_X1);
    parent->SampleQuotient(s_Q0);
    MergeStates(s_Q0, s_X1, q_random);

    X1->freeState(s_X1);
    Q0->freeState(s_Q0);

    return Q1->isValid(q_random);
  }
}
bool Quotient::SampleQuotient(ob::State *state)
{
  return Q1_valid_sampler->sample(state);
}

double Quotient::GetImportance() const
{
  return 0;
}
void Quotient::Print(std::ostream& out) const
{
  out << "[QuotientSpace"<< id << "] ";
  uint sublevel = std::max(1U, level);
  if(parent == nullptr){
    out << "X" << sublevel << "=Q" << sublevel << ": ";
    if( Q1->getStateSpace()->getType() == ob::STATE_SPACE_SE2 ){
      out << "SE(2)";
    }else if( Q1->getStateSpace()->getType() == ob::STATE_SPACE_SE3 ){
      out << "SE(3)";
    }else if( Q1->getStateSpace()->getType() == ob::STATE_SPACE_REAL_VECTOR ){
      out << "R^" << Q1->getStateDimension();
    }else{
      out << "unknown";
    }
  }else{
    out << "X" << sublevel << "=Q" << sublevel << ": ";
    switch (type) {
      case Quotient::RN_RM:
        {
          out << "R^"<< Q0_dimension << " | Q" << level+1 << ": R^" << Q1_dimension << " | X" << level+1 << ": R^" << Q1_dimension-Q0_dimension;
          break;
        }
      case Quotient::SE2_R2:
        {
          out << "R^2 | Q" << level+1 << ": SE(2) | X" << level+1 << ": SO(2)";
          break;
        }
      case Quotient::SE3_R3:
        {
          out << "R^3 | Q" << level+1 << ": SE(3) | X" << level+1 << ": SO(3)";
          break;
        }
      case Quotient::SE3RN_SE3:
        {
          out << "SE(3) | Q" << level+1 << ": SE(3)xR^" << X1_dimension << " | X" << level+1 << ": R^" << X1_dimension;
          break;
        }
      case Quotient::SE3RN_SE3RM:
        {
          out << "SE(3)xR^" << Q0_dimension-6 << " | Q" << level+1 << ": SE(3)xR^"<<Q1_dimension-6 << " | X" << level+1 << ": R^" << X1_dimension;
          break;
        }
      default:
       {
         out << "unknown type: " << type;
       }
    }
  }
}

namespace ompl{
  namespace geometric{
    std::ostream& operator<< (std::ostream& out, const Quotient& quotient_){
      quotient_.Print(out);
      return out;
    }
    // std::ostream& operator<< (std::ostream& out, const Quotient* quotient_){
    //   quotient_->Print(out);
    //   return out;
    // }
  }
}
