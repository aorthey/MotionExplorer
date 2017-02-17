#include <KrisLibrary/planning/KinodynamicCSpace.h>
#include <KrisLibrary/math/random.h>

//class IntegratedKinodynamicCSpace : public KinodynamicCSpace
//{
// public:
//  enum Integrator { Euler, RK4 };
//  IntegratedKinodynamicCSpace(Integrator type=Euler);
//  virtual void Simulate(const State& x0, const ControlInput& u,std::vector<State>& p);
//  ///Compute dx=x'=g(x,u)
//  virtual void XDerivative(const State& x, const ControlInput& u,State& dx)=0;
//  ///Return (deterministically) the time step in dt and the number of
//  ///integration steps in numSteps.
//  virtual void Parameters(const State& x,const ControlInput& u,Real& dt,int& numSteps)=0;
//  Integrator type;
//};

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
//
//  virtual void Simulate(const State& x0, const ControlInput& u,std::vector<State>& p);
//  virtual void SimulateEndpoint(const State& x0, const ControlInput& u,State& x1);
//  virtual EdgePlanner* TrajectoryChecker(const std::vector<State>& p);
//  virtual bool IsValidControl(const State& x,const ControlInput& u);
//  virtual void SampleControl(const State& x,ControlInput& u);
//  virtual void BiasedSampleControl(const State& x,const State& xGoal,ControlInput& u);
//  virtual bool ConnectionControl(const State& x,const State& xGoal,ControlInput& u);
//
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


//class KinodynamicCSpaceSentinel: public KinematicCSpaceAdaptor
//{
//  public:
//    enum Integrator { Euler, RK4 };
//
//    void KinematicCSpaceAdaptor::SampleControl(const State& x,ControlInput& u)
//    {
//        base->SampleNeighborhood(x,maxNeighborhoodRadius,u);
//    }
//    void Simulate(const State& x0, const ControlInput& u,vector<State>& p)
//    {
//
//      
//      IntegrationFunction func(this,u);
//      Real dt,h;
//      int i,numSteps;
//      Parameters(x0,u,dt,numSteps);
//      h = dt/numSteps;
//      State temp;
//      p.push_back(x0);
//      switch(type) {
//      case Euler:
//        for(i=0;i<numSteps;i++) {
//          Euler_step(&func,0,h,p.back(),temp);
//          p.push_back(temp);
//        }
//        break;
//      case RK4:
//        for(i=0;i<numSteps;i++) {
//          RungeKutta4_step(&func,0,h,p.back(),temp);
//          p.push_back(temp);
//        }
//        break;
//      default:
//        FatalError("Unknown integrator type!");
//        break;
//      }
//
//    }
//}


class KinodynamicCSpaceSentinel: public IntegratedKinodynamicCSpace
{
  public:
    KinodynamicCSpaceSentinel(){
      //this->type=RK4; 
      this->type=Euler; 
    }

    virtual void XDerivative(const State& x, const ControlInput& u, State& dx){
      // Lie Algebra Generators
      Matrix4 X1,X2,X3,X4,X5,X6;
      //##########################
      X1(1,2) = -1;
      X1(2,1) = 1;
      //##########################
      X2(0,2) = 1;
      X2(2,0) = -1;
      //##########################
      X3(0,1) = -1;
      X3(1,0) = 1;
      //##########################
      X4(0,3) = 1;
      //##########################
      X5(1,3) = 1;
      //##########################
      X6(2,3) = 1;

      //dx = mul(X1,u(0)) + mul(X2,u(1)) + mul(X3,u(2)) + X4;
      
      Matrix4 tmp = u(0)*X1;

      //dx = x*(u(0)*X1)
    }
    virtual void Parameters(const State& x,const ControlInput& u,Real& dt,int& numSteps){
      dt = 0.001;
      numSteps = 4;
    }
    ///Return an edge planner that checks the simulation trace p for feasibility
    ///Typically, just return new GivenPathEdgePlanner(this,p,tolerance)
    virtual EdgePlanner* TrajectoryChecker(const std::vector<State>& p){
      double tolerance = 1e-3;
      return new GivenPathEdgePlanner(this,p,tolerance);
    }

    virtual bool IsValidControl(const State& x,const ControlInput& u){
      Vector3 u_low_limit(0,-1,-1);
      Vector3 u_up_limit(0,1,1);
      return  (u_low_limit[0] <= u[0] && u[0] <= u_up_limit[0]) &&
              (u_low_limit[1] <= u[1] && u[1] <= u_up_limit[1]) &&
              (u_low_limit[2] <= u[2] && u[2] <= u_up_limit[2]);

    }

    ///Randomly pick a control input
    virtual void SampleControl(const State& x,ControlInput& u){
      double ak = 1;
      u(0) = 0;
      u(1) = Rand(-ak,ak);
      u(2) = Rand(-ak,ak);
    }
     virtual void Sample(Config& x){

     }
     virtual bool IsFeasible(const Config& x){

     }

};
