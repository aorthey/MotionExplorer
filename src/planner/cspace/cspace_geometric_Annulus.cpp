#include "planner/cspace/cspace_geometric_Annulus.h"
#include "planner/cspace/validitychecker/validity_checker_ompl.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include "common.h"
#include "gui/colors.h"
#include <KrisLibrary/GLdraw/drawextra.h>
#include <KrisLibrary/GLdraw/GLColor.h>

#include <ompl/util/Exception.h>

#include <boost/math/constants/constants.hpp>

using namespace boost::math::double_constants;


//############################################################################
class AnnulusStateSampler : public ompl::base::StateSampler
{
public:
		AnnulusStateSampler(const ob::StateSpace *space) : ob::StateSampler(space)
		{
		}

		void sampleUniform(ob::State *state) override
		{
        ob::SO2StateSpace::StateType *stateSO2 = 
          state->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);

        ob::RealVectorStateSpace::StateType *stateRn = 
          state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

        stateSO2->value = rng_.uniformReal(-pi, pi);

        const ob::RealVectorBounds &bounds = space_->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1)->getBounds();
          
        double low2 = bounds.low[0]*bounds.low[0];
        double high2 = bounds.high[0]*bounds.high[0];
        double u = rng_.uniformReal(0,1);
        stateRn->values[0] = sqrtf(u*(high2-low2) + low2);
		}

		void sampleUniformNear(ob::State *state, const ob::State *near, double distance) override
    {
        const ob::SO2StateSpace::StateType *nearSO2 = 
          near->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);
        ob::SO2StateSpace::StateType *stateSO2 = 
          state->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);

        const ob::RealVectorStateSpace::StateType *nearRn = 
          near->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);
        ob::RealVectorStateSpace::StateType *stateRn = 
          state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

        double s = nearSO2->value;
        stateSO2->value = rng_.uniformReal(s-distance, s+distance);

        // const ob::RealVectorBounds &bounds = space_->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1)->getBounds();
          
        const double r = nearRn->values[0];
        double low2 = (r-distance)*(r-distance);
        double high2 = (r+distance)*(r+distance);
        double u = rng_.uniformReal(0,1);
        stateRn->values[0] = sqrtf(u*(high2-low2) + low2);

        space_->enforceBounds(state);
    }
		void sampleGaussian(ob::State *state, const ob::State *mean, double stdDev) override
    {
        throw ompl::Exception("NYI");
    }
};

ob::StateSamplerPtr allocAnnulusStateSampler(const ob::StateSpace *space)
{
    return std::make_shared<AnnulusStateSampler>(space);
}

ob::SpaceInformationPtr GeometricCSpaceOMPLAnnulus::SpaceInformationPtr()
{
    si = BaseT::SpaceInformationPtr();
    const ob::StateSamplerAllocator allocator = allocAnnulusStateSampler;
    si->getStateSpace()->setStateSamplerAllocator(allocator);
    return si;
}

//############################################################################

GeometricCSpaceOMPLAnnulus::GeometricCSpaceOMPLAnnulus(RobotWorld *world_, int robot_idx):
  GeometricCSpaceOMPL(world_, robot_idx)
{
}

void GeometricCSpaceOMPLAnnulus::initSpace()
{
    ob::StateSpacePtr SO2(std::make_shared<ob::SO2StateSpace>());
    ob::StateSpacePtr R1(std::make_shared<ob::RealVectorStateSpace>(1));

    R1->as<ob::RealVectorStateSpace>()->setBounds(radiusInner_, radiusOuter_);

    space = SO2 + R1;
}

void GeometricCSpaceOMPLAnnulus::print(std::ostream& out) const
{
    out << "Annulus Space";
}

bool GeometricCSpaceOMPLAnnulus::IsPlanar()
{
    return false;
}

void GeometricCSpaceOMPLAnnulus::DrawGL(GUIState& state)
{
  glDisable(GL_LIGHTING);
  glEnable(GL_BLEND);

  GLDraw::setColor(black);
  glLineWidth(3);

  const double dstep = 0.01;

  glBegin(GL_LINE_LOOP);
  for(double d = 0; d < 2*M_PI; d+=dstep){
    Vector3 v = ProjectToVector3(d, radiusInner_);
    GLDraw::glVertex3v(v);
  }
  glEnd();

  glBegin(GL_LINE_LOOP);
  for(double d = 0; d < 2*M_PI; d+=dstep){
    Vector3 v = ProjectToVector3(d, radiusOuter_);
    GLDraw::glVertex3v(v);
  }
  glEnd();

  glDisable(GL_BLEND);
  glEnable(GL_LIGHTING);

}

void GeometricCSpaceOMPLAnnulus::ConfigToOMPLState(const Config &q, ob::State *qompl)
{
    double x = q[0];
    double y = q[1];

    double r = sqrt(x*x + y*y);
    double u = atan2(y/r, x/r);

    ob::SO2StateSpace::StateType *qomplSO2 = 
      qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);

    ob::RealVectorStateSpace::StateType *qomplRnSpace = 
      qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    qomplSO2->value = u;
    qomplRnSpace->values[0] = r;
}

Vector3 GeometricCSpaceOMPLAnnulus::ProjectToVector3(double u, double r)
{
    Config q = ProjectToConfig(u, r);
    double x = q[0];
    double y = q[1];
    double z = q[2];
    Vector3 vector(x,y,z);
    return vector;
}

Config GeometricCSpaceOMPLAnnulus::ProjectToConfig(double u, double r)
{
    Config q;q.resize(robot->q.size());q.setZero();

    q[0] = r*cos(u);
    q[1] = r*sin(u);
    q[2] = zOffset_;
    return q;
}

double GeometricCSpaceOMPLAnnulus::OMPLStateToSO2Value(const ob::State *qompl)
{
    const ob::SO2StateSpace::StateType *qomplSO2 = 
      qompl->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);

    return qomplSO2->value;
}

double GeometricCSpaceOMPLAnnulus::OMPLStateToRValue(const ob::State *qompl)
{
    const ob::RealVectorStateSpace::StateType *qomplRnSpace = 
      qompl->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

    return qomplRnSpace->values[0];
}

Config GeometricCSpaceOMPLAnnulus::OMPLStateToConfig(const ob::State *x)
{
    double u = OMPLStateToSO2Value(x);
    double r = OMPLStateToRValue(x);
    return ProjectToConfig(u, r);
}

Vector3 GeometricCSpaceOMPLAnnulus::getXYZ(const ob::State *x)
{
    double u = OMPLStateToSO2Value(x);
    double r = OMPLStateToRValue(x);
    return ProjectToVector3(u, r);
}
