#include "planner.h"
#include "util.h"

MotionPlanner::MotionPlanner(RobotWorld *world, WorldSimulation *sim):
  _world(world),_sim(sim)
{
  _irobot = 0;
  _icontroller = 0;
}
const MultiPath& MotionPlanner::GetPath()
{
  return _path;
}

void MotionPlanner::CheckFeasibility( Robot *robot, SingleRobotCSpace &cspace, Config &q){
  if(!cspace.IsFeasible(q)) {
    cout<<"configuration is infeasible, violated constraints:"<<endl;
    std::cout << q << std::endl;
    vector<bool> infeasible;
    cspace.CheckObstacles(q,infeasible);
    for(size_t i=0;i<infeasible.size();i++){
      //if(!infeasible[i]) cout<<"  OK"<<cspace.ObstacleName(i)<<endl;
      if(infeasible[i]){
        int icq = i - (int)robot->joints.size();
        cout<<"-->"<<cspace.ObstacleName(i) << " | pen-depth: ";
        cout << cspace.collisionQueries[icq].PenetrationDepth() << " | dist: ";
        cout << cspace.collisionQueries[icq].Distance(1e-3,1e-3) << std::endl;
      }
    }
    std::cout << cspace.collisionQueries.size() << std::endl;
    exit(0);
  }
}

bool MotionPlanner::solve(Config &p_init, Config &p_goal, double timelimit, bool shortcutting)
{
  MilestonePath milestone_path;
  _p_init = p_init;
  _p_goal = p_goal;
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Motion Planner:" << std::endl;
  std::cout << "p_init =" << p_init << std::endl;
  std::cout << "p_goal =" << p_goal << std::endl;

  Robot *robot = _world->robots[_irobot];
  robot->UpdateConfig(_p_init);
  util::SetSimulatedRobot(robot,*_sim,_p_init);
  //robot->UpdateConfig(_p_goal);

  _world->InitCollisions();

  WorldPlannerSettings settings;
  settings.InitializeDefault(*_world);
  //settings.robotSettings[0].contactEpsilon = 1e-2;
  //settings.robotSettings[0].contactIKMaxIters = 100;

  SingleRobotCSpace cspace = SingleRobotCSpace(*_world,_irobot,&settings);

  CheckFeasibility( robot, cspace, p_init);
  CheckFeasibility( robot, cspace, p_goal);

  MotionPlannerFactory factory;
  //factory.perturbationRadius = 0.1;
  //factory.type = "rrt";
  //factory.type = "prm";
  //factory.type = "sbl";
  //factory.type = "sblprt";
  //factory.type = "rrt*";
  //factory.type = "lazyrrg*";
  //factory.type = "fmm"; //warning: slows down system 

  //factory.type = "sbl";
  factory.type = "rrt";
  factory.shortcut = shortcutting;

  SmartPointer<MotionPlannerInterface> planner = factory.Create(&cspace,p_init,p_goal);

  HaltingCondition cond;
  cond.foundSolution=true;
  cond.timeLimit = timelimit;

  std::cout << "Start Planning" << std::endl;
  string res = planner->Plan(milestone_path,cond);
  if(milestone_path.edges.empty())
  {
   printf("Planning failed\n");
   _isSolved = false;
  }else{
   printf("Planning succeeded, path has length %g\n",milestone_path.Length());
   _isSolved = true;
  }

  std::cout << std::string(80, '-') << std::endl;

  if(_isSolved){
    double dstep = 0.1;
    Config cur;
    vector<Config> keyframes;
    for(double d = 0; d <= 1; d+=dstep)
    {
      milestone_path.Eval(d,cur);
      keyframes.push_back(cur);
    }
    _path.SetMilestones(keyframes);
    double xtol=0.01;
    double ttol=0.01;
    Robot *robot = _world->robots[_irobot];
    bool res=GenerateAndTimeOptimizeMultiPath(*robot,_path,xtol,ttol);
    return true;
  }else{
    std::cout << "Planner did not find a solution" << std::endl;
    return false;
  }
}

void MotionPlanner::SendCommandStringController(string cmd, string arg)
{
  if(!_sim->robotControllers[_icontroller]->SendCommand(cmd,arg)) {
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "ERROR in controller commander" << std::endl;
    std::cout << cmd << " command  does not work with the robot's controller" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    throw "Controller command not supported!";
  }
}
bool MotionPlanner::SendToController()
{
  if(!_isSolved){ return false; }

  double dstep = 0.1;
  Config q;
  Config dq;

  for(double d = 0; d <= 1; d+=dstep)
  {
    _path.Evaluate(d, q, dq);
    stringstream qstr;
    qstr<<q<<dq;
    string cmd( (d<=0)?("set_qv"):("append_qv") );
    SendCommandStringController(cmd,qstr.str());
  }

  std::cout << "Sending Path to Controller" << std::endl;
  Robot *robot = _world->robots[_irobot];
  util::SetSimulatedRobot(robot,*_sim,_p_init);
  robot->UpdateConfig(_p_goal);
  std::cout << "Done Path to Controller" << std::endl;
  return true;
}

