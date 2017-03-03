#include <Planning/RobotCSpace.h>
#include <vector>

using namespace Math3D;

class CSpaceGoalSetEpsilonNeighborhood: public CSpace
{
  private:
    CSpace *base;
    Config _goal;
    double epsilon;
    double bestDist;
  public:
    CSpaceGoalSetEpsilonNeighborhood(CSpace *base, const Config& goal, double epsilon=0.1):
      base(base),_goal(goal),epsilon(epsilon){
        bestDist = dInf;
      }

    virtual ~CSpaceGoalSetEpsilonNeighborhood() {}

    virtual bool IsFeasible(const Config& q){
      double d = base->Distance(q,_goal);
      if(d<bestDist){
        std::cout << "BEST Distance: " << d << ":" << q << std::endl; 
        std::cout << "               " << _goal << std::endl; 
        bestDist = d;


        //RigidTransform Ta,Tb;
        //ConfigToTransform(q,Ta);
        //ConfigToTransform(_goal,Tb);
        //Real d = Ta.t.distance(Tb.t);
        //Matrix3 Rrel;
        //Rrel.mulTransposeB(Ta.R,Tb.R);
        //AngleAxisRotation aa;
        //aa.setMatrix(Rrel);
        //double wt = 1;
        //double wr = 1;
        //d = Sqrt(d*d*wt + aa.angle*aa.angle*wr);
        //std::cout << " estimated : " << d << std::endl;



      }
      return (d < epsilon);
    }
    virtual void Sample(Config& q){
      //TODO:sample in open epsilon disc
      q = _goal;
    }
    virtual EdgePlanner* LocalPlanner(const Config& a,const Config& b){
      throw("GoalSet Class has no localplanner");
      return NULL;
    }


};

