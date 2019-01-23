#include "common.h"
#include "quotient_cover.h"

using namespace ompl::geometric;
//#############################################################################
//Configuration
//#############################################################################
QuotientCover::Configuration::Configuration(const base::SpaceInformationPtr &si): 
  state(si->allocState())
{}
QuotientCover::Configuration::Configuration(const base::SpaceInformationPtr &si, const ob::State *state_): 
  state(si->cloneState(state_))
{}

QuotientCover::Configuration::~Configuration()
{
}

double QuotientCover::Configuration::GetRadius() const
{
  return openNeighborhoodRadius;
}
void QuotientCover::Configuration::SetRadius(double radius)
{
  openNeighborhoodRadius = radius;
}
void QuotientCover::Configuration::SetOuterRadius(double radius)
{
  openNeighborhoodOuterRadius = radius;
}

void QuotientCover::Configuration::Remove(const base::SpaceInformationPtr &si)
{
  Clear();
  si->freeState(state);
  if(riemannian_center_of_mass != nullptr) si->freeState(riemannian_center_of_mass);
}
void QuotientCover::Configuration::Clear()
{
  number_attempted_expansions = 0;
  number_successful_expansions = 0;

  parent_neighbor = nullptr;
  isSufficientFeasible = false;
  pdf_element = nullptr;
  pdf_necessary_element = nullptr;
  pdf_connectivity_element = nullptr;

  index = -1;
}
void QuotientCover::Configuration::SetPDFElement(void *element_)
{
  pdf_element = element_;
}
void* QuotientCover::Configuration::GetPDFElement()
{
  return pdf_element;
}
void QuotientCover::Configuration::SetNecessaryPDFElement(void *element_)
{
  pdf_necessary_element = element_;
}
void* QuotientCover::Configuration::GetNecessaryPDFElement()
{
  return pdf_necessary_element;
}
void QuotientCover::Configuration::SetConnectivityPDFElement(void *element_)
{
  pdf_connectivity_element = element_;
}
void* QuotientCover::Configuration::GetConnectivityPDFElement()
{
  return pdf_connectivity_element;
}

double QuotientCover::Configuration::GetImportance() const
{
  //return openNeighborhoodRadius + 1.0/goal_distance+1;
  double d = GetRadius();
  return d;
  //return ((double)number_successful_expansions+1)/((double)number_attempted_expansions+2);
  //return 1.0/((double)number_attempted_expansions+1);
}
double QuotientCover::Configuration::GetGoalDistance() const
{
  return goal_distance;
}
ob::State* QuotientCover::Configuration::GetInwardPointingConfiguration() const
{
  if(number_of_neighbors==0){
    if(parent_neighbor==nullptr){
      std::cout << "No neighbors, no parent. Cannot compute an Outward pointing configuration" << std::endl;
      exit(0);
    }
    return parent_neighbor->state;
  }
  return riemannian_center_of_mass;
}


//The riemannian center of mass (RCoM) is the geometric mean of all neighbors
//on the surface of its neighborhood.
//Computing it explicitly is not trivial (see
//https://arxiv.org/abs/1407.2087 | Also named Frechet mean, or Karcher mean). 
//Note that RCoM is not unique (imagine two opposite points on the circle, then we have two
//RCoMs).
//A common approach (first developed by Karcher) is to create a vector
//field pointing towards the mean. This is an alternative defintion for
//the mean in the euclidean case. It generalizes quite nicely to
//manifolds and we can use it to move towards the mean using gradient
//descent. 
//However, using gradient descent might be too time consuming, so here
//we opt for an easier option: incremental computation of RCoM. The idea
//being that we update the RCoM each time a new neighbor arrives. This
//might move us towards an undesired minima, but we will accept that,
//because any minima is good for our application (namely to find
//configurations which point away from the bulk of neighbors, which is
//basically the inverse RCoM, easily computed by interpolating from the
//RCoM along the center to obtain a point antipolar to the RCoM).
//
//NOTE: Please also take a look at https://www.cise.ufl.edu/~salehian/SALEHIAN_H.pdf
//(seems to be solving this exact problem)
//https://link.springer.com/content/pdf/10.1007/978-3-319-22957-7.pdf#page=27
//
void QuotientCover::Configuration::UpdateRiemannianCenterOfMass(ob::SpaceInformationPtr si, Configuration* q_new)
{
  //Case1: There is only a single neighbor => RCoM = neighbor
  if(number_of_neighbors == 0){
    riemannian_center_of_mass = si->cloneState(q_new->state); //clone=alloc+copy=colloc
  }else{
    //Case2: Multiple neighbors, update RCoM by moving along the
    //geodesic between RCoM and q_to. If the total distance between RCoM
    //and q_to is d, then we like to move d*(1/number_of_neighbors)
    //along the geodesic.
    //@TODO just walk through ambient space for now. Not sure how to get the
    //geodesic from here

    si->getStateSpace()->interpolate(riemannian_center_of_mass, q_new->state, (1.0/(number_of_neighbors+1)), riemannian_center_of_mass );
    //Project onto NBH
    double d = si->distance(this->state, riemannian_center_of_mass);
    if(d<1e-10){
      std::cout << "[WARNING]: RIemannian Center of Mass conincides with center" << std::endl;
      //in that case we should try to interpolate constrained to a plane going
      //through 
    }
    si->getStateSpace()->interpolate( this->state, riemannian_center_of_mass, GetRadius()/d, riemannian_center_of_mass);
  }
}

namespace ompl{
  namespace geometric{
    std::ostream& operator<< (std::ostream& out, const QuotientCover::Configuration& q){
      out << "[Configuration]";
      out << q.GetRadius() << std::endl;
      return out;
    }
  }
}
