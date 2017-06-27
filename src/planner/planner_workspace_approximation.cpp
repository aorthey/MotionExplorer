#include "planner/planner_workspace_approximation.h"

PlannerWorkspaceApproximation::PlannerWorkspaceApproximation(Vector3 &init_, Vector3 &goal_, SingleRobotCSpace *inner_, SingleRobotCSpace *outer_):
  init(init_), goal(goal_), inner(inner_), outer(outer_)
{
  inner_radius = fabs(GetRadiusFromRobot(inner->GetRobot()));
  outer_radius = fabs(GetRadiusFromRobot(outer->GetRobot()));

  std::cout << "WorkspaceApproximation" << std::endl;
  std::cout << inner_radius << std::endl;
  std::cout << outer_radius << std::endl;
}

double PlannerWorkspaceApproximation::GetRadiusFromRobot( Robot *r ){
  for(int i = 0; i < r->links.size(); i++){
    if(r->IsGeometryEmpty(i)) continue;
    const GLDraw::GeometryAppearance& a = *r->geomManagers[i].Appearance();
    AABB3D box = a.geom->GetAABB();
    return box.bmin[0];
  }
}


void PlannerWorkspaceApproximation::solve(){
  uint N = 20;
  uint M = 100;

  struct WSphere{
    Vector3 pos;
    double radius;
    double distance_goal;
  };
  auto norm_comp =
      [](const WSphere& e1, const WSphere& e2) 
      { 
        return e1.distance_goal > e2.distance_goal;
      };

  std::priority_queue<WSphere,
                      std::vector<WSphere>,
                      decltype(norm_comp)> spheres(norm_comp);

  if(!IsFeasible(init)){
    std::cout << "Initial state infeasible!" << std::endl;
    return;
  }
  WSphere B0;
  B0.pos = init;
  B0.radius = inner_radius;
  B0.distance_goal = (init-goal).norm();
  spheres.push(B0);

  for(int i = 0; i < M; i++){
    if(spheres.empty()) break;

    WSphere B = spheres.top();
    Vector3 q0 = B.pos;
    std::cout << "feasible sphere: " << q0 << std::endl;
    tree.push_back(q0);
    spheres.pop();
    for(int i = 0; i < N; i++){
      double u = Rand(0.0,1.0);
      double v = Rand(0.0,1.0);
      double theta = 2*M_PI*u;
      double gamma = acos(2*v-1);
      Vector3 q1;
      q1[0] = q0[0]+B.radius*sin(theta)*cos(gamma);
      q1[1] = q0[1]+B.radius*sin(theta)*sin(gamma);
      q1[2] = q0[2]+B.radius*cos(theta);
      if(IsFeasible(q1)){
        WSphere B1;
        B1.pos = q1;
        B1.radius = inner_radius;
        B1.distance_goal = (q1-goal).norm()-inner_radius;
        spheres.push(B1);
      }
    }
  }

}
bool PlannerWorkspaceApproximation::IsFeasible(Vector3 &qq){
  for(int j = 0; j < tree.size(); j++){
    if((tree.at(j)-qq).norm() < inner_radius){
      return false;
    }
  }

  Config q;q.resize(3);
  for(int k = 0; k < 3; q(k)=qq[k],k++);

  // std::cout << "state" << qq << std::endl;
  // std::cout << "inner sphere feasible " << (inner->IsFeasible(q)?"yes":"no") << std::endl;
  // std::cout << "outer sphere feasible " << (outer->IsFeasible(q)?"yes":"no") << std::endl;

  //return isCollisionFree(inner,q) && !isCollisionFree(outer,q);
  return !isCollisionFree(outer,q) && isCollisionFree(inner,q);
}

bool PlannerWorkspaceApproximation::isCollisionFree(SingleRobotCSpace *space, Config q){
  Robot* robot = space->GetRobot();
  robot->UpdateConfig(q);
  robot->UpdateGeometry();

  //same as Singlerobotcspace but ignore other robots
  int id = space->world.RobotID(space->index);
  vector<int> idrobot(1,id);
  vector<int> idothers;
  for(size_t i=0;i<space->world.terrains.size();i++)
    idothers.push_back(space->world.TerrainID(i));
  for(size_t i=0;i<space->world.rigidObjects.size();i++)
    idothers.push_back(space->world.RigidObjectID(i));

  pair<int,int> res = space->settings->CheckCollision(space->world,idrobot,idothers);
  if(res.first >= 0) return false;
  res = space->settings->CheckCollision(space->world,idrobot);
  if(res.first >= 0) return false;
  return true;
}
