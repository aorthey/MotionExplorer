#include "planner/cspace/cspace_geometric_fixedbase.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

GeometricCSpaceOMPLFixedBase::GeometricCSpaceOMPLFixedBase(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
  Nklampt = robot->q.size() - 6;

  if(Nklampt<=0){
    Nompl = 0;
  }else{
    //sometimes the joint space are only fixed joints. In that case OMPL
    //complains that the real vector space is empty. We check here for that case
    //and remove the real vector space
    std::vector<double> minimum, maximum;
    minimum = robot->qMin;
    maximum = robot->qMax;
    assert(minimum.size() == 6+Nklampt);
    assert(maximum.size() == 6+Nklampt);

    //prune dimensions which are smaller than epsilon (for ompl)
    double epsilonSpacing=1e-10;
    Nompl = 0;
    for(uint i = 6; i < 6+Nklampt; i++){

      if(abs(minimum.at(i)-maximum.at(i))>epsilonSpacing){
        klampt_to_ompl.push_back(Nompl);
        ompl_to_klampt.push_back(i);
        Nompl++;
      }else{
        klampt_to_ompl.push_back(-1);
      }
    }
  }
}


void GeometricCSpaceOMPLFixedBase::initSpace()
{

  if(Nompl<=0){
    std::cout << "Fixed Base robot needs to have at least one actuated joint." << std::endl;
    throw "No actuated joints.";
  }

  ob::StateSpacePtr RN = (std::make_shared<ob::RealVectorStateSpace>(Nompl));
  this->space = RN;
  ob::RealVectorStateSpace *cspace = this->space->as<ob::RealVectorStateSpace>();

  std::vector<double> minimum, maximum;
  minimum = robot->qMin;
  maximum = robot->qMax;

  vector<double> lowRn, highRn;
  for(uint i = 0; i < Nompl;i++){
    uint idx = ompl_to_klampt.at(i);
    double min = minimum.at(idx);
    double max = maximum.at(idx);
    lowRn.push_back(min);
    highRn.push_back(max);
  }
  ob::RealVectorBounds boundsRn(Nompl);

  boundsRn.low = lowRn;
  boundsRn.high = highRn;
  boundsRn.check();
  cspace->setBounds(boundsRn);

  //Required to circumvent problems during fine manipulation planning.
  this->space->setLongestValidSegmentFraction(0.001);

}

void GeometricCSpaceOMPLFixedBase::ConfigToOMPLState(const Config &q, ob::State *qompl)
{

  ob::RealVectorStateSpace::StateType *qomplRN = qompl->as<ob::RealVectorStateSpace::StateType>();
  double* qomplRnValues = static_cast<ob::RealVectorStateSpace::StateType*>(qomplRN)->values;
  for(uint i = 0; i < Nklampt; i++){
    int idx = klampt_to_ompl.at(i);
    if(idx<0) continue;
    else qomplRnValues[idx]=q(6+i);
  }
}

Config GeometricCSpaceOMPLFixedBase::OMPLStateToConfig(const ob::State *qompl){
  const ob::RealVectorStateSpace::StateType *qomplRN = qompl->as<ob::RealVectorStateSpace::StateType>();
  Config q = robot->q; //do not change first 6 values (robot might have been translated from origin)
  for(uint i = 0; i < Nompl; i++){
    uint idx = ompl_to_klampt.at(i);
    q(idx) = qomplRN->values[i];
  }
  return q;

}
void GeometricCSpaceOMPLFixedBase::print(std::ostream& out) const
{
  std::cout << "Robot \"" << robot->name << "\":" << std::endl;
  std::cout << "Dimensionality Space            :" << GetDimensionality() << std::endl;
  std::cout << " Configuration Space (klampt) : " << (Nklampt>0?"xR^"+std::to_string(Nklampt):"") << "  [Klampt]"<< std::endl;
  std::cout << " Configuration Space (ompl)   : " << (Nompl>0?"xR^"+std::to_string(Nompl):"") << "  [OMPL]" << std::endl;
}

Vector3 GeometricCSpaceOMPLFixedBase::getXYZ(const ob::State *s)
{
  Config q = OMPLStateToConfig(s);
  robot->UpdateConfig(q);
  robot->UpdateGeometry();
  Vector3 qq;
  Vector3 zero; zero.setZero();
  int lastLink = robot->links.size()-1;

  //NOTE: the world position is zero exactly at the point where link is
  //attached using a joint to the whole linkage. Check where your last fixed
  //joint is positioned, before questioning the validity of this method
  robot->GetWorldPosition(zero, lastLink, qq);

  double x = qq[0];
  double y = qq[1];
  double z = qq[2];
  Vector3 v(x,y,z);
  return v;
}
