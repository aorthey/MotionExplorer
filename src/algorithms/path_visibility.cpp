#include "path_visibility.h"

//Let p,q be two paths [0,1]->M
//Linearsegmentvaliditychecker: 
//  for given s,t \in [0,1] checks if the linear segments between p(s) and q(t)
//  is valid. This is done by calling ompl's checkmotion
//
//  it is used to determine inside a RRT planner to verify if two paths p,q are
//  onetopic, i.e. homotopic by using a linear homotopy deformation.

class LinearSegmentValidityChecker : public ob::StateValidityChecker
{
  public:
    LinearSegmentValidityChecker(const ob::SpaceInformationPtr &si_, const ob::SpaceInformationPtr &si_path_, const ob::PlannerData &pd_in, CSpaceOMPL *cspace_, const std::vector<Vertex> &p1, const std::vector<Vertex> &p2):
      ob::StateValidityChecker(si_), cspace(cspace_), si_path(si_path_)
    {
      std::vector<Config> s1;
      std::vector<Config> s2;

      for(uint k = 0; k < p1.size(); k++){
        const ob::State *state1 = pd_in.getVertex(p1.at(k)).getState();
        Config q1 = cspace->OMPLStateToConfig(state1);
        s1.push_back(q1);
      }
      for(uint k = 0; k < p2.size(); k++){
        const ob::State *state2 = pd_in.getVertex(p2.at(k)).getState();
        Config q2 = cspace->OMPLStateToConfig(state2);
        s2.push_back(q2);
      }

      path1 = PathPiecewiseLinearEuclidean::from_keyframes(s1);
      path1->Normalize();
      path2 = PathPiecewiseLinearEuclidean::from_keyframes(s2);
      path2->Normalize();
    }

    virtual bool isValid(const ob::State* state) const{

      const ob::RealVectorStateSpace::StateType *RnSpace = state->as<ob::RealVectorStateSpace::StateType>();
      double t1 = RnSpace->values[0];
      double t2 = RnSpace->values[1];

      Config q1 = path1->Eval(t1);
      Config q2 = path2->Eval(t2);

      ob::ScopedState<> s1 = cspace->ConfigToOMPLState(q1);
      ob::ScopedState<> s2 = cspace->ConfigToOMPLState(q2);

      bool isfeasible = si_path->checkMotion(s1.get(),s2.get());

      //cspace->SpacePtr()->freeState(s1);
      //cspace->SpacePtr()->freeState(s2);

      return isfeasible;
    }

  private:

    CSpaceOMPL *cspace;
    ob::SpaceInformationPtr si_path;
    PathPiecewiseLinearEuclidean *path1;
    PathPiecewiseLinearEuclidean *path2;

};


PathVisibilityChecker::PathVisibilityChecker(const ob::SpaceInformationPtr si_path_, CSpaceOMPL *cspace_):
si_path(si_path_),cspace(cspace_){
}

bool PathVisibilityChecker::isVisible(const ob::PlannerData& pd, const std::vector<Vertex> &p1, const std::vector<Vertex> &p2)
{
  ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

  //[0,1] x [0,1]
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0);
  bounds.setHigh(1);

  ob::StateSpacePtr space = (std::make_shared<ob::RealVectorStateSpace>(2));
  ob::RealVectorStateSpace *R2 = space->as<ob::RealVectorStateSpace>();

  R2->setBounds(bounds);

  ob::ScopedState<> start(space);
  ob::ScopedState<> goal(space);
  start[0]=start[1]=0.0;
  goal[0]=goal[1]=1.0;

  og::SimpleSetup ss(space);
  const ob::SpaceInformationPtr si_local = ss.getSpaceInformation();

  ss.setStateValidityChecker(std::make_shared<LinearSegmentValidityChecker>(si_local, si_path, pd, cspace, p1, p2));


  ob::PlannerPtr ompl_planner = std::make_shared<og::RRT>(si_local);
  double epsilon_goalregion = 0.01;

  ss.setStartAndGoalStates(start, goal, epsilon_goalregion);
  ss.setPlanner(ompl_planner);
  ss.setup();

  ////set objective to infinite path to just return first solution
  ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
  pdef->setOptimizationObjective( getThresholdPathLength(si_local) );

  double max_planning_time=0.05;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max_planning_time) );

  ss.solve(ptc);
  bool solved = ss.haveExactSolutionPath();

  //if(!si_local->isValid( start.get() )){
  //  const ob::StateValidityCheckerPtr& checker = si_path->getStateValidityChecker();

  //  std::cout << "start state not valid" << std::endl;
  //  std::cout << start << std::endl;
  //  std::cout << "path1:" << std::endl;
  //  for(uint k = 0; k < p1.size(); k++){
  //    const ob::State *state = pd.getVertex(p1.at(k)).getState();
  //    Config q = cspace->OMPLStateToConfig(state);
  //    std::cout << q << std::endl;
  //    std::cout << (checker->isValid(state)?"is valid":"invalid") << std::endl;
  //  }
  //  std::cout << "path2:" << std::endl;
  //  for(uint k = 0; k < p2.size(); k++){
  //    const ob::State *state = pd.getVertex(p2.at(k)).getState();
  //    Config q = cspace->OMPLStateToConfig(state);
  //    std::cout << q << std::endl;
  //    std::cout << (checker->isValid(state)?"is valid":"invalid") << std::endl;
  //  }
  //  exit(0);
  //}
  ompl::msg::setLogLevel(ompl::msg::LOG_INFO);
  return solved;
}
