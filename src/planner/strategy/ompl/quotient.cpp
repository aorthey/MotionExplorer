#include "quotient.h"
#include "planner/cspace/cspace.h"

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

using namespace ompl::geometric;
using namespace ompl::base;


Quotient::Quotient(const ob::SpaceInformationPtr &si, Quotient *previous_):
  ob::Planner(si,"QuotientSpace"), M1(si), M0(si), previous(previous_)
{
  const StateSpacePtr M1_space = M1->getStateSpace();

  id = counter++;
  std::cout << "--- Level " << id << " " << getName() << std::endl;
  setName("Quotient"+std::to_string(id));
  if(previous == nullptr){
    std::cout << "M1 dimension : " << M1_space->getDimension() << " measure: " << M1_space->getMeasure() << std::endl;
    type = ATOMIC_RN;
  }else{
    M0 = previous->getSpaceInformation();
    const StateSpacePtr M0_space = previous->getSpaceInformation()->getStateSpace();

    //C1 = M1 / M0
    const StateSpacePtr C1_space = ComputeQuotientSpace(M1_space, M0_space);

    C1 = std::make_shared<SpaceInformation>(C1_space);
    C1_sampler = C1->allocStateSampler();

    if(M0_space->getDimension()+C1_space->getDimension() != M1_space->getDimension()){
      std::cout << "quotient of state spaces got dimensions wrong." << std::endl;
      exit(0);
    }
    std::cout << "M0 dimension : " << M0_space->getDimension() << " measure: " << M0_space->getMeasure() << std::endl;
    std::cout << "C1 dimension : " << C1_space->getDimension() << " measure: " << C1_space->getMeasure() << std::endl;
    std::cout << "M1 dimension : " << M1_space->getDimension() << " measure: " << M1_space->getMeasure() << std::endl;
    if((M0_space->getMeasure()<=0) ||
       (C1_space->getMeasure()<=0) ||
       (M1_space->getMeasure()<=0)){
      std::cout << "zero-measure quotient space detected. abort." << std::endl;
      exit(0);
    }
    if (!C1_sampler){
      C1_sampler = C1->allocStateSampler();
    }
  }
  if (!M1_valid_sampler){
    M1_valid_sampler = M1->allocValidStateSampler();
  }
  if (!M1_sampler){
    M1_sampler = M1->allocStateSampler();
  }
}

ob::PlannerStatus Quotient::solve(const ob::PlannerTerminationCondition &ptc)
{
  std::cout << "Quotient-Space cannot be solved alone. Use class MultiQuotient." << std::endl;
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
  totalNumberOfSamples = 0;
  graphLength = 0;
  if(previous==nullptr) C1_sampler.reset();
}

uint Quotient::GetNumberOfSampledVertices()
{
  return totalNumberOfSamples;
}
uint Quotient::GetNumberOfVertices()
{
  return 0;
}
uint Quotient::GetNumberOfEdges()
{
  return 0;
}

const StateSpacePtr Quotient::ComputeQuotientSpace(const StateSpacePtr M1, const StateSpacePtr M0)
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
    // (1) M1 Rn       , M0 Rm       , 0<m<n   => C1 = R(n-m)
    // ---- compound:
    // (2) M1 SE2      , M0 R2                 => C1 = SO2
    // (3) M1 SE3      , M0 R3                 => C1 = SO3
    // (4) M1 SE3xRn   , M0 SE3                => C1 = Rn
    // (5) M1 SE3xRn   , M0 R3                 => C1 = SO3xRn
    // (6) M1 SE3xRn   , M0 SE3xRm   , 0<m<n   => C1 = R(n-m)
    ///// M1 SE3      , M0 R3xSO2xSO2         =>C1 = SO2
    ///// M1 R3xS1xS1 , M0 R3                 =>C1 = SO2xSO2

    if(!M1->isCompound()){
      ///##############################################################################/
      //------------------ non-compound cases:
      ///##############################################################################/
      //
      //------------------ (1) M1 = Rn, M0 = Rm, 0<m<n, C1 = R(n-m)
      if( M1->getType() == base::STATE_SPACE_REAL_VECTOR ){
        uint n = M1->getDimension();
        if( M0->getType() == base::STATE_SPACE_REAL_VECTOR ){
          uint m = M0->getDimension();
          if(n>m && m>0){
            type = RN_RM;
          }else{
            std::cout << "not allowed: we need n>m>0, but have " << n << ">" << m << ">0" << std::endl;
            exit(0);
          }
        }else{
          std::cout << "M1 is R^"<<n <<" but state " << M1->getType() << " is not handled." << std::endl;
          exit(0);
        }
      }else{
        std::cout << "M1 is non-compound state, but state " << M1->getType() << " is not handled." << std::endl;
        exit(0);
      }
    }else{
      ///##############################################################################/
      //------------------ compound cases:
      ///##############################################################################/
      //
      //------------------ (2) M1 = SE2, M0 = R2, C1 = SO2
      ///##############################################################################/
      if( M1->getType() == base::STATE_SPACE_SE2 ){
        if( M0->getType() == base::STATE_SPACE_REAL_VECTOR){
          if( M0->getDimension() == 2){
            type = SE2_R2;
          }else{
            std::cout << "M1 is SE2 but state " << M0->getType() << " is of dimension " << M0->getDimension() << std::endl;
            exit(0);
          }
        }else{
          std::cout << "M1 is SE2 but state " << M0->getType() << " is not handled." << std::endl;
          exit(0);
        }
      }
      //------------------ (3) M1 = SE3, M0 = R3, C1 = SO3
      ///##############################################################################/
      else if( M1->getType() == base::STATE_SPACE_SE3 ){
        if( M0->getType() == base::STATE_SPACE_REAL_VECTOR){
          if( M0->getDimension() == 3){
            type = SE3_R3;
          }else{
            std::cout << "M1 is SE3 but state " << M0->getType() << " is of dimension " << M0->getDimension() << std::endl;
            exit(0);
          }
        }else{
          std::cout << "M1 is SE3 but state " << M0->getType() << " is not handled." << std::endl;
          exit(0);
        }
      }
      //------------------ M1 = SE3xRN
      ///##############################################################################/
      else{
        ob::CompoundStateSpace *M1_compound = M1->as<ob::CompoundStateSpace>();
        const std::vector<StateSpacePtr> M1_decomposed = M1_compound->getSubspaces();
        uint M1_subspaces = M1_decomposed.size();
        if(M1_subspaces == 2){
          if(M1_decomposed.at(0)->getType() == base::STATE_SPACE_SE3
              && M1_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR){

            uint n = M1_decomposed.at(1)->getDimension();
            if( M0->getType() == base::STATE_SPACE_SE3 ){
            //------------------ (4) M1 = SE3xRn, M0 = SE3, C1 = Rn
            ///##############################################################################/
              type = SE3RN_SE3;
            }else if(M0->getType() == base::STATE_SPACE_REAL_VECTOR){
            //------------------ (5) M1 = SE3xRn, M0 = R3, C1 = SO3xRN
            ///##############################################################################/
              type = SE3RN_R3;
            }else{
            //------------------ (6) M1 = SE3xRn, M0 = SE3xRm, C1 = R(n-m)
            ///##############################################################################/
              ob::CompoundStateSpace *M0_compound = M0->as<ob::CompoundStateSpace>();
              const std::vector<StateSpacePtr> M0_decomposed = M0_compound->getSubspaces();
              uint M0_subspaces = M0_decomposed.size();
              if(M0_subspaces==2){
                if(M1_decomposed.at(0)->getType() == base::STATE_SPACE_SE3
                    && M1_decomposed.at(1)->getType() == base::STATE_SPACE_REAL_VECTOR){
                  uint m = M0_decomposed.at(1)->getDimension();
                  if(m<n && m>0){
                    type = SE3RN_SE3RM;
                  }else{
                    std::cout << "not allowed: we need n>m>0, but have " << n << ">" << m << ">0" << std::endl;
                    exit(0);
                  }
                }
              }else{
                std::cout << "State compound with " << M0_subspaces << " not handled."<< std::endl;
                exit(0);
              }
            }
          }else{
            std::cout << "State compound " <<  M1_decomposed.at(0)->getType() << " and " <<  M1_decomposed.at(1)->getType() << " not recognized."<< std::endl;
            exit(0);
          }
        }else{
          std::cout << "M1 has " << M1_subspaces << " but we only support 2." << std::endl;
          exit(0);
        }
      }

    }

    StateSpacePtr C1;
    M1_dimension = M1->getDimension();
    M0_dimension = M0->getDimension();

    switch (type) {
      case RN_RM:
        {
          uint N = M1_dimension - M0_dimension;
          C1 = std::make_shared<ob::RealVectorStateSpace>(N);
          C1_dimension = N;

          RealVectorBounds M1_bounds = static_pointer_cast<ob::RealVectorStateSpace>(M1)->getBounds();
          std::vector<double> low; low.resize(N);
          std::vector<double> high; high.resize(N);
          RealVectorBounds C1_bounds(N);
          for(uint k = 0; k < N; k++){
            C1_bounds.setLow(k, M1_bounds.low.at(k+M0_dimension));
            C1_bounds.setHigh(k, M1_bounds.high.at(k+M0_dimension));
          }
          static_pointer_cast<ob::RealVectorStateSpace>(C1)->setBounds(C1_bounds);

          break;
        }
      case SE2_R2:
        {
          C1 = std::make_shared<ob::SO2StateSpace>();
          break;
        }
      case SE3_R3:
        {
          C1 = std::make_shared<ob::SO3StateSpace>();
          break;
        }
      case SE3RN_SE3:
        {
          ob::CompoundStateSpace *M1_compound = M1->as<ob::CompoundStateSpace>();
          const std::vector<StateSpacePtr> M1_decomposed = M1_compound->getSubspaces();

          C1_dimension = M1_decomposed.at(1)->getDimension();

          C1 = std::make_shared<ob::RealVectorStateSpace>(C1_dimension);
          static_pointer_cast<ob::RealVectorStateSpace>(C1)->setBounds( static_pointer_cast<ob::RealVectorStateSpace>(M1_decomposed.at(1))->getBounds() );

          break;
        }
      case SE3RN_R3:
        {
          ob::CompoundStateSpace *M1_compound = M1->as<ob::CompoundStateSpace>();
          const std::vector<StateSpacePtr> M1_decomposed = M1_compound->getSubspaces();
          const std::vector<StateSpacePtr> M1_SE3_decomposed = M1_decomposed.at(0)->as<ob::CompoundStateSpace>()->getSubspaces();

          //const ob::SE3StateSpace *M1_SE3 = M1_SE3_decomposed.at(0)->as<ob::SE3StateSpace>();
          //const ob::SO3StateSpace *M1_SO3 = M1_SE3_decomposed.at(1)->as<ob::SO3StateSpace>();
          const ob::RealVectorStateSpace *M1_RN = M1_decomposed.at(1)->as<ob::RealVectorStateSpace>();
          uint N = M1_RN->getDimension();

          ob::StateSpacePtr SO3(new ob::SO3StateSpace());
          ob::StateSpacePtr RN(new ob::RealVectorStateSpace(N));
          RN->as<ob::RealVectorStateSpace>()->setBounds( M1_RN->getBounds() );

          C1 = SO3 + RN;
          C1_dimension = 3+N;
          break;
        }
      case SE3RN_SE3RM:
        {
          ob::CompoundStateSpace *M1_compound = M1->as<ob::CompoundStateSpace>();
          const std::vector<StateSpacePtr> M1_decomposed = M1_compound->getSubspaces();
          ob::CompoundStateSpace *M0_compound = M0->as<ob::CompoundStateSpace>();
          const std::vector<StateSpacePtr> M0_decomposed = M0_compound->getSubspaces();

          uint N = M1_decomposed.at(1)->getDimension();
          uint M = M0_decomposed.at(1)->getDimension();
          C1_dimension = N-M;
          C1 = std::make_shared<ob::RealVectorStateSpace>(C1_dimension);

          RealVectorBounds M1_bounds = static_pointer_cast<ob::RealVectorStateSpace>(M1_decomposed.at(1))->getBounds();
          std::vector<double> low; low.resize(C1_dimension);
          std::vector<double> high; high.resize(C1_dimension);
          RealVectorBounds C1_bounds(C1_dimension);
          for(uint k = 0; k < C1_dimension; k++){
            C1_bounds.setLow(k, M1_bounds.low.at(k+M));
            C1_bounds.setHigh(k, M1_bounds.high.at(k+M));
          }
          static_pointer_cast<ob::RealVectorStateSpace>(C1)->setBounds(C1_bounds);
          break;
        }
      default:
        {
          std::cout << "unknown type: " << type << std::endl;
          exit(0);
        }

    }
    return C1;

}

void Quotient::mergeStates(const ob::State *qM0, const ob::State *qC1, ob::State *qM1) const
{
  ////input : qM0 \in M0, qC1 \in C1
  ////output: qM1 = qM0 \circ qC1 \in M1
  const StateSpacePtr M1_space = M1->getStateSpace();
  const StateSpacePtr C1_space = C1->getStateSpace();
  const StateSpacePtr M0_space = previous->getSpaceInformation()->getStateSpace();

  switch (type) {
    case RN_RM:
      {
        ob::RealVectorStateSpace::StateType *sM1 = qM1->as<RealVectorStateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *sM0 = qM0->as<RealVectorStateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *sC1 = qC1->as<RealVectorStateSpace::StateType>();

        for(uint k = 0; k < M0_dimension; k++){
          sM1->values[k] = sM0->values[k];
        }
        for(uint k = M0_dimension; k < M1_dimension; k++){
          sM1->values[k] = sC1->values[k-M0_dimension];
        }
        break;
      }
    case SE2_R2:
      {
        ob::SE2StateSpace::StateType *sM1 = qM1->as<SE2StateSpace::StateType>();
        const ob::RealVectorStateSpace::StateType *sM0 = qM0->as<RealVectorStateSpace::StateType>();
        const ob::SO2StateSpace::StateType *sC1 = qC1->as<SO2StateSpace::StateType>();

        sM1->setXY( sM0->values[0], sM0->values[1] );
        sM1->setYaw( sC1->value );

        break;
      }
    case SE3_R3:
      {
        ob::SE3StateSpace::StateType *sM1 = qM1->as<SE3StateSpace::StateType>();
        ob::SO3StateSpace::StateType *sM1_rotation = &sM1->rotation();

        const ob::RealVectorStateSpace::StateType *sM0 = qM0->as<RealVectorStateSpace::StateType>();
        const ob::SO3StateSpace::StateType *sC1 = qC1->as<SO3StateSpace::StateType>();

        sM1->setXYZ( sM0->values[0], sM0->values[1], sM0->values[2]);

        sM1_rotation->x = sC1->x;
        sM1_rotation->y = sC1->y;
        sM1_rotation->z = sC1->z;
        sM1_rotation->w = sC1->w;

        break;
      }
    case SE3RN_R3:
      {
        ob::SE3StateSpace::StateType *sM1_SE3 = qM1->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        ob::SO3StateSpace::StateType *sM1_SO3 = &sM1_SE3->rotation();
        ob::RealVectorStateSpace::StateType *sM1_RN = qM1->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        const ob::RealVectorStateSpace::StateType *sM0 = qM0->as<RealVectorStateSpace::StateType>();
        const ob::SO3StateSpace::StateType *sC1_SO3 = qC1->as<ob::CompoundState>()->as<ob::SO3StateSpace::StateType>(0);
        const ob::RealVectorStateSpace::StateType *sC1_RN = qC1->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        sM1_SE3->setXYZ( sM0->values[0], sM0->values[1], sM0->values[2]);
        sM1_SO3->x = sC1_SO3->x;
        sM1_SO3->y = sC1_SO3->y;
        sM1_SO3->z = sC1_SO3->z;
        sM1_SO3->w = sC1_SO3->w;

        for(uint k = 0; k < C1_dimension-3; k++){
          sM1_RN->values[k] = sC1_RN->values[k];
        }

        break;
      }
    case SE3RN_SE3:
      {
        ob::SE3StateSpace::StateType *sM1_SE3 = qM1->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        ob::SO3StateSpace::StateType *sM1_SE3_rotation = &sM1_SE3->rotation();
        ob::RealVectorStateSpace::StateType *sM1_RN = qM1->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        const ob::SE3StateSpace::StateType *sM0 = qM0->as<SE3StateSpace::StateType>();
        const ob::SO3StateSpace::StateType *sM0_rotation = &sM0->rotation();
        const ob::RealVectorStateSpace::StateType *sC1 = qC1->as<RealVectorStateSpace::StateType>();

        sM1_SE3->setXYZ( sM0->getX(), sM0->getY(), sM0->getZ());
        sM1_SE3_rotation->x = sM0_rotation->x;
        sM1_SE3_rotation->y = sM0_rotation->y;
        sM1_SE3_rotation->z = sM0_rotation->z;
        sM1_SE3_rotation->w = sM0_rotation->w;

        for(uint k = 0; k < C1_dimension; k++){
          sM1_RN->values[k] = sC1->values[k];
        }

        break;
      }
    case SE3RN_SE3RM:{
        ob::SE3StateSpace::StateType *sM1_SE3 = qM1->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        ob::SO3StateSpace::StateType *sM1_SE3_rotation = &sM1_SE3->rotation();
        ob::RealVectorStateSpace::StateType *sM1_RN = qM1->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        const ob::SE3StateSpace::StateType *sM0_SE3 = qM0->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        const ob::SO3StateSpace::StateType *sM0_SE3_rotation = &sM0_SE3->rotation();
        const ob::RealVectorStateSpace::StateType *sM0_RM = qM0->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        const ob::RealVectorStateSpace::StateType *sC1 = qC1->as<RealVectorStateSpace::StateType>();

        sM1_SE3->setXYZ( sM0_SE3->getX(), sM0_SE3->getY(), sM0_SE3->getZ());
        sM1_SE3_rotation->x = sM0_SE3_rotation->x;
        sM1_SE3_rotation->y = sM0_SE3_rotation->y;
        sM1_SE3_rotation->z = sM0_SE3_rotation->z;
        sM1_SE3_rotation->w = sM0_SE3_rotation->w;

        //[X Y Z YAW PITCH ROLL] [1...M-1][M...N-1]
        //SE3                    RN
        uint M = M1_dimension-C1_dimension-6;
        uint N = C1_dimension;

        for(uint k = 0; k < M; k++){
          sM1_RN->values[k] = sM0_RM->values[k];
        }
        for(uint k = M; k < M+N; k++){
          sM1_RN->values[k] = sC1->values[k-M];
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
void Quotient::ExtractC1Subspace( const ob::State* q, ob::State* qC1 ) const
{
  switch(type){
    case RN_RM:
      {
        const ob::RealVectorStateSpace::StateType *sM1 = q->as<RealVectorStateSpace::StateType>();
        ob::RealVectorStateSpace::StateType *sC1 = qC1->as<RealVectorStateSpace::StateType>();

        for(uint k = M0_dimension; k<M1_dimension; k++){
          sC1->values[k-M0_dimension] = sM1->values[k];
        }
        break;
      }
    case SE2_R2:
      {
        const ob::SE2StateSpace::StateType *sM1 = q->as<SE2StateSpace::StateType>();
        ob::SO2StateSpace::StateType *sC1 = qC1->as<SO2StateSpace::StateType>();
        sC1->value = sM1->getYaw();
        break;
      }
    case SE3_R3:
      {
        const ob::SE3StateSpace::StateType *sM1 = q->as<SE3StateSpace::StateType>();
        const ob::SO3StateSpace::StateType *sM1_SO3 = &sM1->rotation();

        ob::SO3StateSpace::StateType *sC1_SO3 = qC1->as<SO3StateSpace::StateType>();

        sC1_SO3->x = sM1_SO3->x;
        sC1_SO3->y = sM1_SO3->y;
        sC1_SO3->z = sM1_SO3->z;
        sC1_SO3->w = sM1_SO3->w;

        break;
      }
    case SE3RN_R3:
      {
        const ob::SE3StateSpace::StateType *sM1_SE3 = q->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        const ob::SO3StateSpace::StateType *sM1_SO3 = &sM1_SE3->rotation();
        const ob::RealVectorStateSpace::StateType *sM1_RN = q->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        ob::SO3StateSpace::StateType *sC1_SO3 = qC1->as<ob::CompoundState>()->as<SO3StateSpace::StateType>(0);
        ob::RealVectorStateSpace::StateType *sC1_RN = qC1->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        sC1_SO3->x = sM1_SO3->x;
        sC1_SO3->y = sM1_SO3->y;
        sC1_SO3->z = sM1_SO3->z;
        sC1_SO3->w = sM1_SO3->w;
        for(uint k = 0; k < C1_dimension-3; k++){
          sC1_RN->values[k] = sM1_RN->values[k];
        }

        break;
      }
    case SE3RN_SE3:
      {
        const ob::RealVectorStateSpace::StateType *sM1_RN = q->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);
        ob::RealVectorStateSpace::StateType *sC1 = qC1->as<RealVectorStateSpace::StateType>();

        for(uint k = 0; k < C1_dimension; k++){
          sC1->values[k] = sM1_RN->values[k];
        }

        break;
      }
    case SE3RN_SE3RM:{
        const ob::RealVectorStateSpace::StateType *sM1_RN = q->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        const ob::RealVectorStateSpace::StateType *sC1 = qC1->as<RealVectorStateSpace::StateType>();

        uint N = M1_dimension - C1_dimension - 6;
        for(uint k = N; k < M1_dimension-6; k++){
          sC1->values[k-N] = sM1_RN->values[k];
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

void Quotient::ExtractM0Subspace( const ob::State* q, ob::State* qM0 ) const
{

  switch(type){
    case RN_RM:
      {
        const ob::RealVectorStateSpace::StateType *sM1 = q->as<RealVectorStateSpace::StateType>();
        ob::RealVectorStateSpace::StateType *sM0 = qM0->as<RealVectorStateSpace::StateType>();

        for(uint k = 0; k<M0_dimension; k++){
          sM0->values[k] = sM1->values[k];
        }
        break;
      }
    case SE2_R2:
      {
        const ob::SE2StateSpace::StateType *sM1 = q->as<SE2StateSpace::StateType>();
        ob::RealVectorStateSpace::StateType *sM0 = qM0->as<RealVectorStateSpace::StateType>();
        sM0->values[0] = sM1->getX();
        sM0->values[1] = sM1->getY();
        break;
      }
    case SE3_R3:
      {
        const ob::SE3StateSpace::StateType *sM1 = q->as<SE3StateSpace::StateType>();
        ob::RealVectorStateSpace::StateType *sM0 = qM0->as<RealVectorStateSpace::StateType>();

        sM0->values[0] = sM1->getX();
        sM0->values[1] = sM1->getY();
        sM0->values[2] = sM1->getZ();

        break;
      }
    case SE3RN_R3:
      {
        const ob::SE3StateSpace::StateType *sM1_SE3 = q->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        //const ob::SO3StateSpace::StateType *sM1_SO3 = &sM1_SE3->rotation();
        //const ob::RealVectorStateSpace::StateType *sM1_RN = q->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        ob::RealVectorStateSpace::StateType *sM0 = qM0->as<RealVectorStateSpace::StateType>();

        sM0->values[0] = sM1_SE3->getX();
        sM0->values[1] = sM1_SE3->getY();
        sM0->values[2] = sM1_SE3->getZ();

        break;
      }
    case SE3RN_SE3:
      {
        const ob::SE3StateSpace::StateType *sM1_SE3 = q->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        const ob::SO3StateSpace::StateType *sM1_SE3_rotation = &sM1_SE3->rotation();

        ob::SE3StateSpace::StateType *sM0 = qM0->as<SE3StateSpace::StateType>();
        ob::SO3StateSpace::StateType *sM0_rotation = &sM0->rotation();

        sM0->setXYZ( sM1_SE3->getX(), sM1_SE3->getY(), sM1_SE3->getZ());
        sM0_rotation->x = sM1_SE3_rotation->x;
        sM0_rotation->y = sM1_SE3_rotation->y;
        sM0_rotation->z = sM1_SE3_rotation->z;
        sM0_rotation->w = sM1_SE3_rotation->w;

        break;
      }
    case SE3RN_SE3RM:
      {
        const ob::SE3StateSpace::StateType *sM1_SE3 = q->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        const ob::SO3StateSpace::StateType *sM1_SE3_rotation = &sM1_SE3->rotation();
        const ob::RealVectorStateSpace::StateType *sM1_RN = q->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        ob::SE3StateSpace::StateType *sM0_SE3 = qM0->as<ob::CompoundState>()->as<SE3StateSpace::StateType>(0);
        ob::SO3StateSpace::StateType *sM0_rotation = &sM0_SE3->rotation();
        ob::RealVectorStateSpace::StateType *sM0_RN = qM0->as<ob::CompoundState>()->as<RealVectorStateSpace::StateType>(1);

        sM0_SE3->setXYZ( sM1_SE3->getX(), sM1_SE3->getY(), sM1_SE3->getZ());
        sM0_rotation->x = sM1_SE3_rotation->x;
        sM0_rotation->y = sM1_SE3_rotation->y;
        sM0_rotation->z = sM1_SE3_rotation->z;
        sM0_rotation->w = sM1_SE3_rotation->w;

        for(uint k = 0; k < M0_dimension-6; k++){
          sM0_RN->values[k] = sM1_RN->values[k];
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

const ob::SpaceInformationPtr &Quotient::getC1() const{
  return C1;
}

bool Quotient::SampleC1(ob::State *s){
  C1_sampler->sampleUniform(s);
  return true;
}
bool Quotient::HasSolution()
{
  return hasSolution;
}

bool Quotient::Sample(ob::State *q_random)
{
  totalNumberOfSamples++;
  if(previous == nullptr){
    return M1_valid_sampler->sample(q_random);
  }else{
    //Adjusted sampling function: Sampling in G0 x C1

    ob::SpaceInformationPtr M0 = previous->getSpaceInformation();
    base::State *s_C1 = C1->allocState();
    base::State *s_M0 = M0->allocState();

    C1_sampler->sampleUniform(s_C1);
    previous->SampleGraph(s_M0);
    mergeStates(s_M0, s_C1, q_random);

    C1->freeState(s_C1);
    M0->freeState(s_M0);

    return M1->isValid(q_random);
  }
}

bool Quotient::SampleGraph(ob::State *q_random)
{
  std::cout << "SampleGraph: NYI" << std::endl;
  exit(0);
}

double Quotient::GetSamplingDensity(){
  //double N = (double)totalNumberOfSamples;
  double N = (double)GetNumberOfVertices();
  //return N;
  //@TODO: needs a more formal definition of sampling density
  if(previous==nullptr){
    return N/((double)si_->getSpaceMeasure());
  }else{
    return N/((double)si_->getSpaceMeasure());
    //return N/(previous->GetGraphLength()*C1->getSpaceMeasure());
  }
}

double Quotient::GetGraphLength(){
  return graphLength;
}

namespace ompl{
  namespace geometric{
    std::ostream& operator<< (std::ostream& out, const Quotient& qtnt){
      if(qtnt.previous == nullptr){
        out << "M0: ";
        if( qtnt.M1->getStateSpace()->getType() == ob::STATE_SPACE_SE2 ){
          out << "SE(2)" << std::endl;
        }else if( qtnt.M1->getStateSpace()->getType() == ob::STATE_SPACE_SE3 ){
          out << "SE(3)" << std::endl;
        }else if( qtnt.M1->getStateSpace()->getType() == ob::STATE_SPACE_REAL_VECTOR ){
          out << "R^" << qtnt.M1_dimension << std::endl;
        }else{
          out << "unknown" << std::endl;
        }
      }
      switch (qtnt.type) {
        case Quotient::RN_RM:
          {
            out << "M0: R^"<< qtnt.M0_dimension << " | M1: R^" << qtnt.M1_dimension << " | C1: R^" << qtnt.M1_dimension-qtnt.M0_dimension<< std::endl;
            break;
          }
        case Quotient::SE2_R2:
          {
            out << "M0: SE(2) | M1: R^2 | C1: SO(2)" << std::endl;
            break;
          }
        case Quotient::SE3_R3:
          {
            out << "M0: SE(3) | M1: R^3 | C1: SO(3)" << std::endl;
            break;
          }
        case Quotient::SE3RN_SE3:
          {
            out << "M0: SE(3)xR^" << qtnt.C1_dimension << " | M1: SE(3) | C1: R^" << qtnt.C1_dimension << std::endl;
            break;
          }
        case Quotient::SE3RN_SE3RM:
          {
            out << "M0: SE(3)xR^" << qtnt.M0_dimension-6 << " | M1: SE(3)xR^"<<qtnt.M1_dimension-6 << " | C1: R^" << qtnt.C1_dimension << std::endl;
            break;
          }
        default:
         {
           out << "unknown type: " << qtnt.type << std::endl;
         }

      }
      return out;

    }
  }
}

//std::string typeToString(StateSpaceType type)
//{
//  std::string out;
//  if(type==base::STATE_SPACE_REAL_VECTOR){
//
//  }else if(type==base::STATE_SPACE_SO2){
//  }else if(type==base::STATE_SPACE_SO3){
//  }else if(type==base::STATE_SPACE_SE2){
//  }else if(type==base::STATE_SPACE_SE3){
//  }else if(type==base::STATE_SPACE_TIME){
//  }else if(type==base::STATE_SPACE_DISCRETE){
//  }else{
//    out = "UNKNOWN";
//  }
//  return out;
//}
