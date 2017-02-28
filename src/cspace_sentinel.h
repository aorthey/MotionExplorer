#include <KrisLibrary/planning/KinodynamicCSpace.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math/diffeq.h>
#include <vector>

using namespace Math3D;

//class KinematicCSpaceAdaptor : public KinodynamicCSpace
//{
// public:
//  KinematicCSpaceAdaptor(CSpace* base);
//  virtual ~KinematicCSpaceAdaptor() {}
//
//  //pass-throughs to base space
//  virtual void Sample(Config& x) { base->Sample(x); }
//  virtual void SampleNeighborhood(const Config& c,Real r,Config& x) { base->SampleNeighborhood(c,r,x); }
//  virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) { return base->LocalPlanner(a,b); }
//  virtual bool IsFeasible(const Config& x) { return base->IsFeasible(x); }
//  virtual Real Distance(const Config& x, const Config& y) { return base->Distance(x,y); }
//  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) { base->Interpolate(x,y,u,out); }
//  virtual void Midpoint(const Config& x,const Config& y,Config& out) { base->Midpoint(x,y,out); }
//  virtual EdgePlanner* TrajectoryChecker(const std::vector<State>& p);
//
//  NEEDS TO BE IMPLEMENTED:
//  virtual void Simulate(const State& x0, const ControlInput& u,std::vector<State>& p);
//  virtual bool IsValidControl(const State& x,const ControlInput& u);
//  virtual void SampleControl(const State& x,ControlInput& u);
//
//  NEEDS TO BE IMPLEMENTED FOR BI-DIR PLANNING
//  virtual bool ConnectionControl(const State& x,const State& xGoal,ControlInput& u);
//
//  MIGHT BE USEFUL
//  virtual void BiasedSampleControl(const State& x,const State& xGoal,ControlInput& u);
//  virtual void SimulateEndpoint(const State& x0, const ControlInput& u,State& x1);
//  virtual bool ReverseControl(const State& x0,const State& x1,ControlInput& u);
//  virtual bool ReverseSimulate(const State& x1, const ControlInput& u,std::vector<State>& p);
//  virtual bool IsValidReverseControl(const State& x1,const ControlInput& u);
//  virtual void SampleReverseControl(const State& x,ControlInput& u);
//  virtual void BiasedSampleReverseControl(const State& x1,const State& xDest,ControlInput& u);
//  virtual void Properties(PropertyMap& map) const;
//
//  CSpace* base;
//  Real maxNeighborhoodRadius;
//};

//Should include all IntegratedkinodynamicCspace functions + the functions from
//the kinematic CSpace pointer + the dynamics
class KinodynamicCSpaceSentinelAdaptor: public KinematicCSpaceAdaptor
{
  public:
    enum Integrator { Euler, RK4 };
    Integrator type;

    KinodynamicCSpaceSentinelAdaptor(CSpace *base);

    ////pass-throughs to base space
    //virtual void Sample(Config& x) { base->Sample(x); }
    virtual void Sample(Config& x) { base->Sample(x);x(2)=1;x(4)=0;x(5)=0; }
    //virtual void SampleNeighborhood(const Config& c,Real r,Config& x) { base->SampleNeighborhood(c,r,x); }
    //virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) { return base->LocalPlanner(a,b); }
    //virtual bool IsFeasible(const Config& x) { return base->IsFeasible(x); }
    //virtual Real Distance(const Config& x, const Config& y) { return base->Distance(x,y); }
    //virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out) { base->Interpolate(x,y,u,out); }
    //virtual void Midpoint(const Config& x,const Config& y,Config& out) { base->Midpoint(x,y,out); }

    // Reimplementation because of too simple implementation in Kinematiccspaceadaptor
    virtual void Simulate(const State& x0, const ControlInput& u,std::vector<State>& p);
    virtual void SimulateEndpoint(const State& x0, const ControlInput& u,State& x1);
    virtual void BiasedSampleControl(const State& x,const State& xGoal,ControlInput& u);

    //required implementation
    virtual EdgePlanner* TrajectoryChecker(const std::vector<State>& p);
    virtual bool IsValidControl(const State& x,const ControlInput& u);
    virtual void SampleControl(const State& x,ControlInput& u);

    //new functions to be used with struct IntegratorFunction 
    virtual Matrix4 SE3Derivative(const Matrix4& x_SE3, const ControlInput& u);
    virtual void Parameters(const State& x,const ControlInput& u,Real& dt,int& numSteps);
    //virtual State SE3ToState(const Matrix4& x_SE3);
    Matrix4 StateToSE3(const State& x);
    void SE3ToState(State& x, const Matrix4& x_SE3);
    Matrix4 MatrixExponential(const Matrix4& x);

    virtual bool ReverseSimulate(const State& x1, const ControlInput& u,std::vector<State>& p);

    virtual bool ConnectionControl(const State& x,const State& xGoal,ControlInput& u) { return false; }
    virtual bool ReverseControl(const State& x0,const State& x1,ControlInput& u) { return false; }
    virtual void BiasedSampleReverseControl(const State& x1,const State& xDest,ControlInput& u);
    //virtual bool ReverseSimulate(const State& x1, const ControlInput& u,std::vector<State>& p) { return false; }

//  virtual void SampleReverseControl(const State& x,ControlInput& u);
//  virtual void BiasedSampleReverseControl(const State& x1,const State& xDest,ControlInput& u);

};

