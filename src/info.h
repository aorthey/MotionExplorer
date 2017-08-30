#pragma once

#include <stdio.h>
#include <Modeling/Paths.h>
#include <Modeling/MultiPath.h>
#include <KrisLibrary/planning/Path.h>
#include <KrisLibrary/planning/KinodynamicPath.h>

class Info
{
  public:
    
    Info()
    {
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "Information Module initialized" << std::endl;
      std::cout << std::string(80, '-') << std::endl;
    }

    void operator()(const MilestonePath &path)
    {
      std::cout <<  path.Length() << std::endl;
    }
    void operator()(const KinodynamicMilestonePath &path)
    {
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "----- KinodynamicMilestonePath Start ------ " << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "Milestones  : " << path.milestones.size() << std::endl;
      std::cout << "Controls    : " << path.controls.size() << std::endl;
      std::cout << "Paths       : " << path.paths.size() << std::endl;
      //std::cout << "Edges       : " << path.edges.size() << std::endl;
      //std::cout << path.edges.at(0) << std::endl;
      //std::cout << path.edges.at(0)->Space() << std::endl;
      //std::cout <<  path.PathLength() << std::endl;
        // std::vector<State> milestones;
        //   std::vector<ControlInput> controls;
        //     std::vector<std::vector<State> > paths;
        //       std::vector<SmartPointer<EdgePlanner> > edges;
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "----- KinodynamicMilestonePath End ------ " << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      std::cout << std::endl;


    }
    void operator()(const MultiPath &path)
    {
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "----- MultiPath Start ------ " << std::endl;
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "Time start    : " << path.StartTime() << std::endl;
      std::cout << "Time end      : " << path.EndTime() << std::endl;
      std::cout << "Path duration : "<< path.Duration() << std::endl;
      std::cout << "Path valid    : "<< ((path.IsValid())?("True"):("False"))<< std::endl;
      std::cout << std::string(80, '-') << std::endl;
      std::cout << "----- MultiPath End ------ " << std::endl;
      std::cout << std::string(80, '-') << std::endl;
    }

    void operator()(RobotWorld *world){
      int ids = world->NumIDs();

      std::cout << std::string(80, '-') << std::endl;

      std::cout << "RobotWorld Info" << std::endl;

      std::cout << std::string(80, '-') << std::endl;
      for(int itr = 0; itr <= ids; itr++){
        std::cout << "[" << itr << "] " << world->GetName(itr) << std::endl;
      }
      std::cout << std::string(80, '-') << std::endl;

      std::vector<SmartPointer<Robot> > robots = world->robots;
      std::vector<SmartPointer<Terrain> > terrains = world->terrains;
      for (std::vector<SmartPointer<Robot> >::iterator it = robots.begin() ; it != robots.end(); ++it){
        std::cout << "Robot " << (*it)->name << std::endl;

        std::cout << std::string(80, '-') << std::endl;
        vector<string> linkNames = (*it)->linkNames;
        vector<RobotLink3D> links = (*it)->links;
        assert( links.size() == linkNames.size() );
        std::cout << "#Links: " << links.size() << std::endl;
        for(uint i = 0; i < links.size(); i++){
          std::cout<< "Link[" << i << "] " << linkNames[i] << " mass " << links[i].mass << std::endl;
        }

        std::cout << std::string(80, '-') << std::endl;
        vector<RobotJoint> joints = (*it)->joints;
        std::cout << "#Joints: " << joints.size() << std::endl;
        for(uint i = 0; i< joints.size(); i++){
          std::cout<< "Joint[" << i << "] linkidx " << joints[i].linkIndex << " type ";
            switch(joints[i].type){
              case RobotJoint::Floating: std::cout << "floating"; break;
              case RobotJoint::Weld: std::cout << "--"; break;
              case RobotJoint::Normal: std::cout << "actuated"; break;
              case RobotJoint::Spin: std::cout << "infinite rotating"; break;
              default: std::cout << "UNKNOWN"; break;
            }
          std::cout << std::endl;
        }

        std::cout << std::string(80, '-') << std::endl;
        vector<RobotJointDriver> drivers = (*it)->drivers;
        vector<string> dnames = (*it)->driverNames;
        std::cout << "#Drivers: " << drivers.size() << std::endl;
        for(uint i = 0; i < drivers.size(); i++){
          double ql = drivers[i].qmin;
          double qu = drivers[i].qmax;
          double vl = drivers[i].vmin;
          double vu = drivers[i].vmax;
          double al = drivers[i].amin;
          double au = drivers[i].amax;
          double tl = drivers[i].tmin;
          double tu = drivers[i].tmax;
          std::cout << "Driver[" << drivers[i].linkIndices.at(0) << "] ";

          std::cout << ql << "<=   q <=" << qu << " | "
                    << vl << "<=  dq <=" << vu << " | "
                    << al << "<=  ddq <=" << au << " | "
                    << tl << "<=  trq <=" << tu << "   ";
          std::cout << "("<< dnames[i] <<")"<<std::endl;
        }

        uint Neffective = 0;
        for(int i = 0; i < (*it)->qMin.size(); i++){
          double qL = (*it)->qMin[i];
          double qU = (*it)->qMax[i];
          if(fabs(qU-qL) > 1e-8) Neffective++;
          //std::cout << "#qlimit [" << i << "] " << (*it)->qMin[i] << " - " <<  (*it)->qMax[i] << std::endl;
        }
        //SE(3) element: X Y Z yaw pitch roll
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "#config q=" << (*it)->q << std::endl;
        std::cout << "#X Y Z          : " << (*it)->q[0] << ","  << (*it)->q[1] << "," << (*it)->q[2]<< std::endl;
        std::cout << "#Yaw Pitch Roll : " << (*it)->q[3] << ","  << (*it)->q[4] << "," << (*it)->q[5]<< std::endl;
        std::cout << "#qmin " << (*it)->qMin << std::endl;
        std::cout << "#qmax " << (*it)->qMax << std::endl;
        std::cout << "Dimensionality          : " << (*it)->qMin.size() << std::endl;
        std::cout << "Effective dimensionality: " << Neffective << " (removing zero measure dimensions)" << std::endl;
        std::cout << std::string(80, '-') << std::endl;

        for(int i = 0; i < (*it)->qMin.size(); i++){
          double qL = (*it)->qMin[i];
          double qU = (*it)->qMax[i];
          double vL = (*it)->velMin[i];
          double vU = (*it)->velMax[i];
          double aL = -(*it)->accMax[i];
          double aU = (*it)->accMax[i];
          if(fabs(qU-qL) > 1e-8) 
          std::cout << "#q= [" << i << "] " << qL << "<=   q <=" << qU << " | "
                                            << vL << "<=  dq <=" << vU << " | "
                                            << aL << "<= ddq <=" << aU << std::endl;
        }

      }//for all robots

    }
};

