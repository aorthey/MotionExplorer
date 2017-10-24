#include "util.h"
#include <KrisLibrary/utils/stringutils.h>

namespace util{
  void SetSimulatedRobot( Robot *robot, WorldSimulation &sim, Config &q)
  {
    robot->UpdateConfig(q);
    int robotidx = 0;
    ODERobot *simrobot = sim.odesim.robot(robotidx);
    simrobot->SetConfig(q);
  }

  std::string GetCurrentTimeString()
  {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    ostringstream os;

    int yyyy = now->tm_year + 1900;
    int mm = now->tm_mon + 1;
    int dd = now->tm_mday;

    os << yyyy << "_" << setfill('0') << setw(2) << mm << "_" << setfill('0') << setw(2) << dd;
    return os.str();
  }

  void PrintCurrentTime()
  {
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );

    printf ( "The current date/time is: %s\n", asctime (now) );

  }
  std::string GetApplicationFolder()
  {
    //std::string name = getenv("USER");
    std::string pwd = getenv("PWD");

    //return "/home/"+name+"/git/orthoklampt/";
    return pwd+"/../";
  }
  std::string GetDataFolder()
  {
    using namespace boost::filesystem;
    path cur = current_path();
    return cur.string()+"/../data";
  }

  bool StartsWith(const std::string &s, const char* prefix){
    return ::StartsWith(s.c_str(),prefix);
  }
  bool StartsWith(const std::string &s, const std::string &prefix){
    return ::StartsWith(s.c_str(),prefix.c_str());
  }

  std::string RemoveStringBeginning(const std::string &s, const std::string &prefix){
    std::string out = s.substr(prefix.size()+1,s.size()-(prefix.size()+1));
    return out;
  }
}
