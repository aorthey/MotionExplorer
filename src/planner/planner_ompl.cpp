#include "planner_ompl.h"

#include "cspace_sentinel.h"
#include "liegroupintegrator.h"
#include "planner/planner_workspace_approximation.h"


void MotionPlannerOMPL::SerializeTree(ob::PlannerData &pd)
{
  //pd.decoupleFromPlanner();
  std::cout << "serializing tree with " << pd.numVertices() << " vertices" << std::endl;
  std::cout << "                  and " << pd.numEdges() << " edges" << std::endl;
  pd.toBoostGraph();

  _stree.clear();
  for(uint i = 0; i < pd.numVertices(); i++){
    ob::PlannerDataVertex v = pd.getVertex(i);
    const ob::State* s = v.getState();
    const ob::SE3StateSpace::StateType *sSE3 = s->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
    double x = sSE3->getX();
    double y = sSE3->getY();
    double z = sSE3->getZ();
    SerializedTreeNode snode;
    Config q;q.resize(6);q.setZero();
    q(0)=x;
    q(1)=y;
    q(2)=z;
    snode.position = q;

    //std::vector<uint> edgeList;
    //uint Nedges = pd.getIncomingEdges (i, edgeList);
     
    std::vector<uint> edgeList;
    pd.getEdges(i, edgeList);
    //std::cout << "Node " << i << " has " << Nedges << " edges" << std::endl;
    for(int j = 0; j < edgeList.size(); j++){
      ob::PlannerDataVertex w = pd.getVertex(edgeList.at(j));
      const ob::State* sw = w.getState();
      const ob::SE3StateSpace::StateType *swSE3 = sw->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0);
      double xw = swSE3->getX();
      double yw = swSE3->getY();
      double zw = swSE3->getZ();

      Vector3 dvw( xw-x, yw-y, zw-z);
      snode.directions.push_back(dvw);
    }



    _stree.push_back(snode);
    
  }
}

MotionPlannerOMPL::MotionPlannerOMPL(RobotWorld *world):
  MotionPlanner(world)
{
}

void PostRunEvent(const ob::PlannerPtr &planner, ot::Benchmark::RunProperties &run, GeometricCSpaceOMPL *cspace)
{
  static uint pid = 0;

  //run["some extra property name INTEGER"] = "some value";
  // The format of added data is string key, string value pairs,
  // with the convention that the last word in string key is one of
  // REAL, INTEGER, BOOLEAN, STRING. (this will be the type of the field
  // when the log file is processed and saved as a database).
  // The values are always converted to string.
  ob::SpaceInformationPtr si = planner->getSpaceInformation();
  ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
  bool solved = pdef->hasExactSolution();

  double dg = pdef->getSolutionDifference();
  std::cout << dg << std::endl;

  if(solved){
    std::cout << "Found Solution at run " << pid << std::endl;
    const ob::PathPtr &pp = pdef->getSolutionPath();
    oc::PathControl path_control = static_cast<oc::PathControl&>(*pp);
    og::PathGeometric path = path_control.asGeometric();

    vector<Config> keyframes;
    for(int i = 0; i < path.getStateCount(); i++)
    {
      ob::State *state = path.getState(i);
      Config cur = cspace->OMPLStateToConfig(state);
      keyframes.push_back(cur);
    }
    std::string sfile = "ompl_"+std::to_string(pid)+".xml";
    std::cout << "Saving keyframes"<< std::endl;
    //Save(keyframes, sfile.c_str());
  }else{
    std::cout << "Run " << pid << " no solution" << std::endl;

  }
  pid++;

}

// class WorkspaceApproximationPlannerValidityChecker : public ob::StateValidityChecker
// {
//   public:
//     WorkspaceApproximationPlannerValidityChecker(const ob::SpaceInformationPtr &si, CSpace *inner_, CSpace *outer_):
//       ob::StateValidityChecker(si),inner(inner_),outer(outer_) {};

//     virtual bool isValid(const ob::State* state) const{
//       //const ob::StateSpacePtr space = si_->getStateSpace();
//       const ob::RealVectorStateSpace::StateType *RnSpace = state->as<ob::RealVectorStateSpace::StateType>();

//       double* qs = RnSpace->values;

//       Config q;q.resize(3);
//       for(int i = 0; i < 3; i++) q[i]=qs[i];
//       // std::cout << "state" << q << std::endl;
//       // std::cout << "inner sphere feasible " << (inner->IsFeasible(q)?"yes":"no") << std::endl;
//       // std::cout << "outer sphere feasible " << (outer->IsFeasible(q)?"yes":"no") << std::endl;
//       // static int kk = 0;
//       // if(kk++>5) exit(0);

//       bool feasible = inner->IsFeasible(q) && !outer->IsFeasible(q);
//       return feasible;

//     }
//     // bool intersectsSpheres(Config &q) const{
//     //   double radius = 0.1;
//     //   for(int i = 0; i < spheres.size(); i++){
//     //     Config s = spheres.at(i);
//     //     if((s-q).norm() < radius){
//     //       return true;
//     //     }
//     //   }
//     //   return false;
//     // }

//     CSpace *inner;
//     CSpace *outer;
// };
// class WorkspaceApproximationPlannerPropagator : public oc::StatePropagator
// {
// public:

//     WorkspaceApproximationPlannerPropagator(oc::SpaceInformationPtr si, Robot *robot_) : 
//         oc::StatePropagator(si.get()), robot(robot_)
//     {
//     }
//     virtual void propagate(const ob::State *state, const oc::Control* control, const double duration, ob::State *result) const override{
//       const ob::RealVectorStateSpace::StateType *RnSpace = state->as<ob::RealVectorStateSpace::StateType>();
//       const double *ucontrol = control->as<oc::RealVectorControlSpace::ControlType>()->values;
//       double* qs = RnSpace->values;
//       Config q;q.resize(3);
//       for(int i = 0; i < 3; i++) q[i]=qs[i];

//       double radius = 0.1;

//       const ob::RealVectorStateSpace::StateType *resultRn = result->as<ob::RealVectorStateSpace::StateType>();
//       double* qr = resultRn->values;
//       qr[0] = q[0]+radius*sin(ucontrol[0])*cos(ucontrol[1]);
//       qr[1] = q[1]+radius*sin(ucontrol[0])*sin(ucontrol[1]);
//       qr[2] = q[2]+radius*cos(ucontrol[0]);

//     }

//     Robot *robot;
//     CSpaceOMPL *ompl_space;
// };

void MotionPlannerOMPL::WorkspaceApproximationPlanner(PlannerInput &input){

  //ob::StateSpacePtr space(std::make_shared<ob::RealVectorStateSpace>(3));

  //ob::RealVectorBounds boundsRn(3);
  //boundsRn.setLow(-100);
  //boundsRn.setHigh(100);
  //space->as<ob::RealVectorStateSpace>()->setBounds(boundsRn);

  //ob::ScopedState<> start(space);
  //ob::ScopedState<> goal(space);
  //for(int i = 0; i < 3; i++){
  //  start[i] = input.q_init[i];
  //  goal[i] = input.q_goal[i];
  //}

  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*_world);

  SingleRobotCSpace sphere_inner = SingleRobotCSpace(*_world,1,&worldsettings);
  SingleRobotCSpace sphere_outer = SingleRobotCSpace(*_world,2,&worldsettings);

  Vector3 init,goal;
  for(int i = 0; i < 3; i++){
    init[i] = input.q_init[i];
    goal[i] = input.q_goal[i];
  }
  PlannerWorkspaceApproximation planner_workspace(init,goal,&sphere_inner,&sphere_outer);
  planner_workspace.solve();
  std::vector<Vector3> tree = planner_workspace.tree;
  std::cout << " tree contains  " << tree.size() << " vertices" << std::endl;

  for(uint i = 0; i < tree.size(); i++){
    Vector3 s = tree.at(i);

    WorkspaceApproximationElement w;
    w.pos = Vector3(s[0],s[1],s[2]);
    w.inner_radius = planner_workspace.inner_radius;
    w.outer_radius = planner_workspace.outer_radius;

    output.workspace.elements.push_back(w);
  }

  //oc::RealVectorControlSpacePtr control_space = std::make_shared<oc::RealVectorControlSpace>(space, 2);
  //ob::RealVectorBounds boundsC(2);

  //boundsC.setLow(0);
  //boundsC.setHigh(M_PI);

  //boundsC.setLow(0,0);
  //boundsC.setHigh(0,2*M_PI);
  //boundsC.setLow(1,0);
  //boundsC.setHigh(1,M_PI);

  //boundsC.check();
  //control_space->setBounds(boundsC);

  //oc::SimpleSetup ss(control_space);
  //oc::SpaceInformationPtr si = ss.getSpaceInformation();

  //ss.setStateValidityChecker(std::make_shared<WorkspaceApproximationPlannerValidityChecker>(si,&sphere_inner,&sphere_outer));
  //ss.setStatePropagator(std::make_shared<WorkspaceApproximationPlannerPropagator>(si,sphere_inner.GetRobot()));
  //si->setStateValidityCheckingResolution(0.1);

  ////###########################################################################
  //// choose planner
  ////###########################################################################
  //ob::PlannerPtr ompl_planner = std::make_shared<oc::KPIECE1>(si);

  //double epsilon_goalregion = input.epsilon_goalregion;

  //ss.setStartAndGoalStates(start, goal, epsilon_goalregion);
  //ss.setup();
  //ss.setPlanner(ompl_planner);
  //ss.getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new WorkspaceProject0r(ss.getStateSpace())));

  ////set objective to infinite path to just return first solution
  //ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
  //pdef->setOptimizationObjective( getThresholdPathLengthObj(si) );

  //double solution_time = dInf;
  //double max_planning_time= input.max_planning_time;
  //ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max_planning_time) );

  //ob::PlannerStatus status = ss.solve(ptc);

  ////###########################################################################
  //// extract roadmap
  ////###########################################################################

  //oc::PlannerData pd(si);
  //ss.getPlannerData(pd);

  ////create workspaceapproximationelements

  //std::cout << "serializing tree with " << pd.numVertices() << " vertices" << std::endl;
  //std::cout << "                  and " << pd.numEdges() << " edges" << std::endl;
  ////pd.toBoostGraph();

  //for(uint i = 0; i < pd.numVertices(); i++){
  //  ob::PlannerDataVertex v = pd.getVertex(i);
  //  const ob::State* state = v.getState();

  //  //std::vector<uint> edgeList;
  //  //pd.getEdges(i, edgeList);

  //  const ob::RealVectorStateSpace::StateType *st = state->as<ob::RealVectorStateSpace::StateType>();
  //  double* s = st->values;

  //  WorkspaceApproximationElement w;
  //  w.pos = Vector3(s[0],s[1],s[2]);
  //  w.inner_radius = 0.3;
  //  w.outer_radius = 1.5;

  //  output.workspace.elements.push_back(w);
  //  
  //}


}


bool MotionPlannerOMPL::solve(PlannerInput &input_)
{
  input = input_;

  Config p_init = input.q_init;
  Config p_goal = input.q_goal;
  std::string algorithm = input.name_algorithm;

  robot->UpdateConfig(p_init);
  this->_world->InitCollisions();
  std::cout << input << std::endl;


  //###########################################################################
  // Setup Klampt CSpace
  //###########################################################################

  WorldPlannerSettings worldsettings;
  worldsettings.InitializeDefault(*_world);

  SingleRobotCSpace kcspace = SingleRobotCSpace(*_world,_irobot,&worldsettings);
  if(!IsFeasible( robot, kcspace, p_goal)) return false;
  if(!IsFeasible( robot, kcspace, p_init)) return false;

  if(algorithm=="workspace"){
    WorkspaceApproximationPlanner(input);
    return true;
  }

  //
  //GeometricCSpaceOMPL: Space = Configuration manifold; control space = tangent
  //space of configuration manifold, i.e. control happens in velocity space
  //
  //KinodynamicCSpaceOMPL: Space = tangent bundle of configuration manifold;
  //control space = tangent space of tangent bundle, i.e. control happens in
  //acceleration space, i.e. we can control torques of revolute joints, forces
  //of prismatic joints, and any additional booster/thruster which act directly
  //on the se(3) component
  //

  CSpaceFactory factory(input);

  //GeometricCSpaceOMPL* cspace = factory.MakeGeometricCSpace(robot, &kcspace);
  KinodynamicCSpaceOMPL* cspace = factory.MakeKinodynamicCSpace(robot, &kcspace);

  //###########################################################################
  // Config init,goal to OMPL start/goal
  //###########################################################################

  ob::ScopedState<> start = cspace->ConfigToOMPLState(p_init);
  ob::ScopedState<> goal  = cspace->ConfigToOMPLState(p_goal);
  std::cout << start << std::endl;
  std::cout << goal << std::endl;

  oc::SimpleSetup ss(cspace->ControlSpacePtr());//const ControlSpacePtr
  oc::SpaceInformationPtr si = ss.getSpaceInformation();

  ss.setStateValidityChecker(cspace->StateValidityCheckerPtr(si));
  ss.setStatePropagator(cspace->StatePropagatorPtr(si));

  //###########################################################################
  // choose planner
  //###########################################################################
  ob::PlannerPtr ompl_planner;

  if(algorithm=="ompl:rrt") ompl_planner = std::make_shared<oc::RRT>(si);
  else if(algorithm=="ompl:sst") ompl_planner = std::make_shared<oc::SST>(si);
  else if(algorithm=="ompl:pdst") ompl_planner = std::make_shared<oc::PDST>(si);
  else if(algorithm=="ompl:kpiece") ompl_planner = std::make_shared<oc::KPIECE1>(si);
  else{
    std::cout << "Planner algorithm " << algorithm << " is unknown." << std::endl;
    return false;
  }

  //###########################################################################
  // setup and projection
  //###########################################################################
  double epsilon_goalregion = input.epsilon_goalregion;

  ss.setStartAndGoalStates(start, goal, epsilon_goalregion);
  ss.setup();
  ss.setPlanner(ompl_planner);
  ss.getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SE3Project0r(ss.getStateSpace())));

  //set objective to infinite path to just return first solution
  ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
  pdef->setOptimizationObjective( getThresholdPathLengthObj(si) );

  //###########################################################################
  // solve
  //
  // termination condition: 
  //    reached duration or found solution in epsilon-neighborhood
  //###########################################################################
  bool solved = false;
  double solution_time = dInf;
  double max_planning_time= input.max_planning_time;
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max_planning_time) );

  //###########################################################################
  // benchmark instead
  //###########################################################################
  // ot::Benchmark benchmark(ss, "BenchmarkSnakeTurbine");
  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::PDST>(si)));
  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::SST>(si)));
  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::KPIECE1>(si)));
  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::RRT>(si)));

  // ot::Benchmark::Request req;
  // req.maxTime = max_planning_time;
  // req.maxMem = 10000.0;
  // req.runCount = 100;
  // req.displayProgress = true;

  // benchmark.setPostRunEvent(std::bind(&PostRunEvent, std::placeholders::_1, std::placeholders::_2, &cspace));
  
  // benchmark.benchmark(req);
  // benchmark.saveResultsToFile();

  // std::string file = "ompl_benchmark";
  // std::string res = file+".log";
  // benchmark.saveResultsToFile(res.c_str());

  // std::string cmd = "ompl_benchmark_statistics.py "+file+".log -d "+file+".db";
  // std::system(cmd.c_str());
  // cmd = "cp "+file+".db"+" ../data/benchmarks/";
  // std::system(cmd.c_str());

  //###########################################################################
  // solve
  //###########################################################################

  ob::PlannerStatus status = ss.solve(ptc);
  solved = ss.haveExactSolutionPath();

  //###########################################################################
  // extract roadmap
  //###########################################################################

  oc::PlannerData pd(si);
  ss.getPlannerData(pd);
  SerializeTree(pd);

  //###########################################################################
  // extract solution path if solved
  //###########################################################################

  solved = ss.haveSolutionPath();
  //return approximate solution
  if (solved)
  {
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Found solution:" << std::endl;
    std::cout << " exact solution       : " << (pdef->hasExactSolution()? "Yes":"No")<< std::endl;
    std::cout << " approximate solution : " << (pdef->hasApproximateSolution()? "Yes":"No")<< std::endl;

    double dg = pdef->getSolutionDifference();
    std::cout << " solution difference  : " << dg << std::endl;
    oc::PathControl path_control = ss.getSolutionPath();
    path_control.interpolate();

    //og::PathGeometric path = path_control.asGeometric();
    std::cout << "Path Length     : " << path_control.length() << std::endl;
    //std::cout << "Path Milestones : " << path.getStateCount() << std::endl;

    std::vector<oc::Control*> controls = path_control.getControls();

    //ControlDimension K = N+6
    uint K = cspace->GetControlDimensionality();

    std::vector<Vector> torques_and_time;
    std::cout << "Controls:" << std::endl;
    std::cout <<K << "x" << controls.size() << std::endl;
    for(int i = 0; i < controls.size(); i++){
      oc::RealVectorControlSpace::ControlType* ccv = static_cast<oc::RealVectorControlSpace::ControlType *>(controls.at(i));

      double time = ccv->values[K-1];
      Vector qt;qt.resize(K);

      qt(K-1) = time;
      for(int k = 0; k < K; k++){
        qt(k) = ccv->values[k];
      }
      torques_and_time.push_back(qt);
    }

    //og::PathSimplifier shortcutter(si);
    //shortcutter.shortcutPath(path);

    std::vector<ob::State *> states = path_control.getStates();
    std::vector<Config> keyframes;
    for(int i = 0; i < states.size(); i++)
    {
      ob::State *state = states.at(i);//path.getState(i);
      Config cc = cspace->OMPLStateToConfig(state);
      std::vector<Real> curd = std::vector<Real>(cc);
      //extract only position
      std::vector<Real> curhalf(curd.begin(),curd.begin()+int(0.5*curd.size()));
      Config cur(curhalf);

      keyframes.push_back(cur);
    }

    uint istep = max(int(keyframes.size()/10.0),1);
    for(int i = 0; i < keyframes.size(); i+=istep)
    {
      std::cout << i << "/" << keyframes.size() << " : "  <<  keyframes.at(i) << std::endl;
    }
    std::cout << keyframes.size() << "/" << keyframes.size() << " : "  <<  keyframes.back() << std::endl;

    output.SetTorques(torques_and_time);
    output.SetKeyframes(keyframes);
    output.SetTree(_stree);
    std::cout << std::string(80, '-') << std::endl;
  }else{
    std::cout << "No solution found" << std::endl;
  }

  return solved;
}

