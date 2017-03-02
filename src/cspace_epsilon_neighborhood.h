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

