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

bool MotionPlanner::solve(Config &p_init, Config &p_goal)
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

  SingleRobotCSpace2 cspace = SingleRobotCSpace(*_world,_irobot,&settings);
  //std::cout << "CSPACE COLLISION PAIRS: " << cspace.collisionPairs.size() << std::endl;
  //cspace.FixDof(6,0);
  //cspace.FixDof(7,0);
  //cspace.FixDof(10,0);
  //cspace.FixDof(11,0);
  //cspace.IgnoreCollisions(7,9);
  //cspace.IgnoreCollisions(7,11);
  //cspace.IgnoreCollisions(7,13);
  //cspace.ignoreCollisions.push_back(pair<int,int>(7,9));
  //cspace.ignoreCollisions.push_back(pair<int,int>(7,10));
  //cspace.ignoreCollisions.push_back(pair<int,int>(7,11));
  //cspace.ignoreCollisions.push_back(pair<int,int>(7,12));
  //cspace.ignoreCollisions.push_back(pair<int,int>(7,13));
  //cspace.ignoreCollisions.push_back(pair<int,int>(0,11));
  //cspace.ignoreCollisions.push_back(pair<int,int>(0,6));
  //cspace.IgnoreCollisions(6,7);
  //cspace.IgnoreCollisions(7,8);
  //cspace.IgnoreCollisions(8,9);
  //cspace.IgnoreCollisions(1,2);
  //cspace.IgnoreCollisions(2,3);
  //cspace.IgnoreCollisions(5,11);
  //for(int i = 0; i < robot->links.size(); i++){
  //  //std::cout << "[" << i << "/" << robot->links.size() << "] " << std::endl;
  //  for(int j = 0; j < robot->links.size(); j++){
  //    cspace.IgnoreCollisions(i,j);
  //  }
  //}

  //std::cout << "CSPACE COLLISION PAIRS: " << cspace.collisionPairs.size() << std::endl;

  //ContactCSpace cspace(freeSpace);
  //for(size_t i=0;i<cspace.fixedDofs.size();i++) 
  //{
  //  int k=cspace.fixedDofs[i];
  //  cspace.ignoreCollisions.push_back(pair<int,int>(objectid,_world->RobotLinkID(_irobot,k)));
  //}

  if(!cspace.IsFeasible(p_init)) {
    cout<<"Start configuration is infeasible, violated constraints:"<<endl;
    std::cout << p_init << std::endl;
    vector<bool> infeasible;
    cspace.CheckObstacles(p_init,infeasible);
    for(size_t i=0;i<infeasible.size();i++){
      if(!infeasible[i]) cout<<"  OK"<<cspace.ObstacleName(i)<<endl;
      if(infeasible[i]){
        int icq = i - (int)robot->joints.size();
        cout<<"-->"<<cspace.ObstacleName(i) << " | "<< endl;
        cout << cspace.collisionQueries[icq].PenetrationDepth() << std::endl;
        cout << cspace.collisionQueries[icq].Distance(1e-3,1e-3) << std::endl;
      }
    }
    std::cout << cspace.collisionQueries.size() << std::endl;
    return false;
  }

  if(!cspace.IsFeasible(p_goal)) {
    cout<<"Goal configuration is infeasible, violated constraints:"<<endl;
    vector<bool> infeasible;
    cspace.CheckObstacles(p_goal,infeasible);
    for(size_t i=0;i<infeasible.size();i++)
      if(infeasible[i]) cout<<"  "<<cspace.ObstacleName(i)<<endl;
    return false;
  }

  MotionPlannerFactory factory;
  const int PLANNER_MAX_ITERS = 10000;
  //factory.perturbationRadius = 0.1;
  //factory.type = "rrt";
  //factory.type = "prm";
  //factory.type = "sbl";
  //factory.type = "sblprt";
  //factory.type = "rrt*";
  //factory.type = "lazyrrg*";
  //factory.type = "fmm"; //warning: slows down system 

  factory.type = "prm";
  factory.shortcut = true;

  int iters = PLANNER_MAX_ITERS;
  SmartPointer<MotionPlannerInterface> planner = factory.Create(&cspace,p_init,p_goal);

  HaltingCondition cond;
  cond.foundSolution=false;
  cond.timeLimit = 10;

  std::cout << "Start Planning" << std::endl;
  string res = planner->Plan(milestone_path,cond);
  if(milestone_path.edges.empty())
  {
   printf("Planning failed\n");
  }else{
   printf("Planning succeeded, path has length %g\n",milestone_path.Length());
  }

  // PlanMore
  //_isSolved = false;
  //while(iters > 0) {
  //  planner->PlanMore();
  //  iters--;
  //  if(planner->IsSolved()) {
  //    planner->GetSolution(milestone_path);
  //    _isSolved = true;
  //    break;
  //  }
  //}

  std::cout << std::string(80, '-') << std::endl;

  if(_isSolved){
    std::cout << "Planner converged after " << PLANNER_MAX_ITERS-iters << "/" << PLANNER_MAX_ITERS << " iterations." << std::endl;
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

