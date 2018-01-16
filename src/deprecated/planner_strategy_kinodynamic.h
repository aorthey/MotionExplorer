// void PostRunEvent(const ob::PlannerPtr &planner, ot::Benchmark::RunProperties &run, GeometricCSpaceOMPL *cspace)
// {
//   static uint pid = 0;

//   //run["some extra property name INTEGER"] = "some value";
//   // The format of added data is string key, string value pairs,
//   // with the convention that the last word in string key is one of
//   // REAL, INTEGER, BOOLEAN, STRING. (this will be the type of the field
//   // when the log file is processed and saved as a database).
//   // The values are always converted to string.
//   ob::SpaceInformationPtr si = planner->getSpaceInformation();
//   ob::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
//   bool solved = pdef->hasExactSolution();

//   double dg = pdef->getSolutionDifference();
//   std::cout << dg << std::endl;

//   if(solved){
//     std::cout << "Found Solution at run " << pid << std::endl;
//     const ob::PathPtr &pp = pdef->getSolutionPath();
//     oc::PathControl path_control = static_cast<oc::PathControl&>(*pp);
//     og::PathGeometric path = path_control.asGeometric();

//     vector<Config> keyframes;
//     for(uint i = 0; i < path.getStateCount(); i++)
//     {
//       ob::State *state = path.getState(i);
//       Config cur = cspace->OMPLStateToConfig(state);
//       keyframes.push_back(cur);
//     }
//     std::string sfile = "ompl_"+std::to_string(pid)+".xml";
//     std::cout << "Saving keyframes"<< std::endl;
//     //Save(keyframes, sfile.c_str());
//   }else{
//     std::cout << "Run " << pid << " no solution" << std::endl;

//   }
//   pid++;

// }


//bool MotionPlannerOMPL::solve_kinodynamically(CSpaceOMPL *cspace){
//  Config p_init = input.q_init;
//  Config p_goal = input.q_goal;
//  std::string algorithm = input.name_algorithm;
//  //###########################################################################
//  // Config init,goal to OMPL start/goal
//  //###########################################################################
//
//  ob::ScopedState<> start = cspace->ConfigToOMPLState(p_init);
//  ob::ScopedState<> goal  = cspace->ConfigToOMPLState(p_goal);
//  std::cout << start << std::endl;
//  std::cout << goal << std::endl;
//
//  oc::SimpleSetup ss(cspace->ControlSpacePtr());//const ControlSpacePtr
//  oc::SpaceInformationPtr si = ss.getSpaceInformation();
//
//  ss.setStateValidityChecker(cspace->StateValidityCheckerPtr(si));
//  ss.setStatePropagator(cspace->StatePropagatorPtr(si));
//
//  //###########################################################################
//  // choose planner
//  //###########################################################################
//
//  ob::PlannerPtr ompl_planner;
//
//  if(algorithm=="ompl:rrt") ompl_planner = std::make_shared<oc::RRT>(si);
//  else if(algorithm=="ompl:sst") ompl_planner = std::make_shared<oc::SST>(si);
//  else if(algorithm=="ompl:pdst") ompl_planner = std::make_shared<oc::PDST>(si);
//  else if(algorithm=="ompl:kpiece") ompl_planner = std::make_shared<oc::KPIECE1>(si);
//  else{
//    std::cout << "Planner algorithm " << algorithm << " is unknown." << std::endl;
//    exit(0);
//    return false;
//  }
//
//  ob::ParamSet psi = si->params();
//  psi.print(std::cout);
//
//  //###########################################################################
//  // setup and projection
//  //###########################################################################
//  double epsilon_goalregion = input.epsilon_goalregion;
//
//  ss.setStartAndGoalStates(start, goal, epsilon_goalregion);
//  ss.setPlanner(ompl_planner);
//  ss.setup();
//  ss.getStateSpace()->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SE3Project0r(ss.getStateSpace())));
//
//  //set objective to infinite path to just return first solution
//  ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
//  pdef->setOptimizationObjective( getThresholdPathLengthObj(si) );
//
//  //###########################################################################
//  // solve
//  //
//  // termination condition: 
//  //    reached duration or found solution in epsilon-neighborhood
//  //###########################################################################
//  bool solved = false;
//  double max_planning_time= input.max_planning_time;
//  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max_planning_time) );
//
//  //###########################################################################
//  // benchmark instead
//  //###########################################################################
//  // ot::Benchmark benchmark(ss, "BenchmarkSnakeTurbine");
//  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::PDST>(si)));
//  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::SST>(si)));
//  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::KPIECE1>(si)));
//  // benchmark.addPlanner(ob::PlannerPtr(std::make_shared<oc::RRT>(si)));
//
//  // ot::Benchmark::Request req;
//  // req.maxTime = max_planning_time;
//  // req.maxMem = 10000.0;
//  // req.runCount = 100;
//  // req.displayProgress = true;
//
//  // benchmark.setPostRunEvent(std::bind(&PostRunEvent, std::placeholders::_1, std::placeholders::_2, &cspace));
//  
//  // benchmark.benchmark(req);
//  // benchmark.saveResultsToFile();
//
//  // std::string file = "ompl_benchmark";
//  // std::string res = file+".log";
//  // benchmark.saveResultsToFile(res.c_str());
//
//  // std::string cmd = "ompl_benchmark_statistics.py "+file+".log -d "+file+".db";
//  // std::system(cmd.c_str());
//  // cmd = "cp "+file+".db"+" ../data/benchmarks/";
//  // std::system(cmd.c_str());
//
//  //###########################################################################
//  // solve
//  //###########################################################################
//
//  //ob::PlannerStatus status = 
//  ss.solve(ptc);
//  solved = ss.haveExactSolutionPath();
//
//  //###########################################################################
//  // extract roadmap
//  //###########################################################################
//
//  oc::PlannerData pd(si);
//  ss.getPlannerData(pd);
//
//  SerializeTree(pd, cspace);
//  output.SetTree(_stree);
//
//  //###########################################################################
//  // extract solution path if solved
//  //###########################################################################
//
//  solved = ss.haveSolutionPath();
//  //return approximate solution
//  if (solved)
//  {
//    std::cout << std::string(80, '-') << std::endl;
//    std::cout << "Found solution:" << std::endl;
//    std::cout << " exact solution       : " << (pdef->hasExactSolution()? "Yes":"No")<< std::endl;
//    std::cout << " approximate solution : " << (pdef->hasApproximateSolution()? "Yes":"No")<< std::endl;
//
//    double dg = pdef->getSolutionDifference();
//    std::cout << " solution difference  : " << dg << std::endl;
//    oc::PathControl path_control = ss.getSolutionPath();
//    path_control.interpolate();
//
//    //og::PathGeometric path = path_control.asGeometric();
//    std::cout << "Path Length     : " << path_control.length() << std::endl;
//
//    std::vector<oc::Control*> controls = path_control.getControls();
//
//    //ControlDimension K = N+6
//    uint K = cspace->GetControlDimensionality();
//
//    std::vector<Vector> torques_and_time;
//    std::cout << "Controls:" << std::endl;
//    std::cout <<K << "x" << controls.size() << std::endl;
//    for(uint i = 0; i < controls.size(); i++){
//      oc::RealVectorControlSpace::ControlType* ccv = static_cast<oc::RealVectorControlSpace::ControlType *>(controls.at(i));
//
//      double time = ccv->values[K-1];
//      Vector qt;qt.resize(K);
//
//      qt(K-1) = time;
//      for(uint k = 0; k < K; k++){
//        qt(k) = ccv->values[k];
//      }
//      torques_and_time.push_back(qt);
//    }
//
//    //og::PathSimplifier shortcutter(si);
//    //shortcutter.shortcutPath(path);
//
//    std::vector<ob::State *> states = path_control.getStates();
//    std::vector<Config> keyframes;
//    for(uint i = 0; i < states.size(); i++)
//    {
//      ob::State *state = states.at(i);//path.getState(i);
//      Config cc = cspace->OMPLStateToConfig(state);
//      std::vector<Real> curd = std::vector<Real>(cc);
//      //extract only position
//      std::vector<Real> curhalf(curd.begin(),curd.begin()+int(0.5*curd.size()));
//      Config cur(curhalf);
//
//      keyframes.push_back(cur);
//    }
//
//    uint istep = max(int(keyframes.size()/10.0),1);
//    for(uint i = 0; i < keyframes.size(); i+=istep)
//    {
//      std::cout << i << "/" << keyframes.size() << " : "  <<  keyframes.at(i) << std::endl;
//    }
//    std::cout << keyframes.size() << "/" << keyframes.size() << " : "  <<  keyframes.back() << std::endl;
//
//    output.SetTorques(torques_and_time);
//    output.SetKeyframes(keyframes);
//    std::cout << std::string(80, '-') << std::endl;
//  }else{
//    std::cout << "No solution found" << std::endl;
//  }
//
//  return solved;
//}


