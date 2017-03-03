#include <KrisLibrary/planning/KinodynamicCSpace.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/math3d/primitives.h>
#include <KrisLibrary/math3d/rotation.h>
#include <KrisLibrary/math/diffeq.h>
#include <Planning/RobotCSpace.h>
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
    virtual void Sample(Config& x) 
    { 
      base->Sample(x);
      //x(2)=1;x(4)=0;x(5)=0;
      //std::cout << "RANDOM SAMPLE" << x << std::endl;
    }

    virtual Real Distance(const Config& x, const Config& y) { 
      //return base->Distance(x,y); 
      //Config xpos;xpos.resize(3);xpos(0)=x(0);xpos(1)=x(1);xpos(2)=x(2);
      //Config ypos;ypos.resize(3);ypos(0)=y(0);ypos(1)=y(1);ypos(2)=y(2);
      //return base->Distance(xpos,ypos); 
      RigidTransform Ta,Tb;
      ConfigToTransform(x,Ta);
      ConfigToTransform(y,Tb);
      Real d = Ta.t.distance(Tb.t);
      Matrix3 Rrel;
      Rrel.mulTransposeB(Ta.R,Tb.R);
      AngleAxisRotation aa;
      aa.setMatrix(Rrel);
      double wt = 1;
      double wr = 0.25;
      d = Sqrt(d*d*wt + aa.angle*aa.angle*wr);
      //std::cout << " estimated : " << d << std::endl;
      return d;
    }

    //virtual void SampleNeighborhood(const Config& c,Real r,Config& x) { base->SampleNeighborhood(c,r,x); }
    //virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b) { return base->LocalPlanner(a,b); }
    //virtual bool IsFeasible(const Config& x) { return base->IsFeasible(x); }
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
    virtual Matrix4 SE3Derivative(const ControlInput& u);
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

