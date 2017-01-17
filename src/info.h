#pragma once

#include <stdio.h>
#include <Modeling/Paths.h>
#include <Modeling/MultiPath.h>

class Info
{
  public:
    
    Info()
    {
      std::cout << "Information Module initialized" << std::endl;
    }

    void operator()(MilestonePath &path)
    {
      std::cout <<  path.Length() << std::endl;
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
        for(int i = 0; i< links.size(); i++){
          std::cout<< "Link[" << i << "] " << linkNames[i] << " mass " << links[i].mass << std::endl;
        }

        //std::cout << std::string(80, '-') << std::endl;
        //vector<RobotJoint> joints = (*it)->joints;
        //for(int i = 0; i< joints.size(); i++){
        //  std::cout<< "Joint[" << i << "] linkidx " << joints[i].linkIndex << " baseidx " << joints[i].baseIndex << " type " << joints[i].type << std::endl;
        //}

        std::cout << std::string(80, '-') << std::endl;
        vector<RobotJointDriver> drivers = (*it)->drivers;
        vector<string> dnames = (*it)->driverNames;
        for(int i = 0; i< drivers.size(); i++){
          std::cout<< "Joint[" << i << "] " << dnames[i] << " qmin " << drivers[i].qmin << " qmax " << drivers[i].qmax << std::endl;
        }

        std::cout << std::string(80, '-') << std::endl;
        std::cout << "#config q=" << (*it)->q << std::endl;
        std::cout << "#qmin " << (*it)->qMin << std::endl;
        std::cout << "#qmax " << (*it)->qMax << std::endl;
        std::cout << std::string(80, '-') << std::endl;
      }//for all robots

    }
};

