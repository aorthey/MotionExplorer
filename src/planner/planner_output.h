#pragma once

class PlannerOutput{

  private:
    double time;
    uint nodes;

    //tree swath
    SerializedTree _stree;

    //path
    std::vector<Config> q;
    std::vector<Config> dq;
    std::vector<Config> ddq;
    std::vector<Vector> torques;

  public:
    PlannerOutput(){};

    void SetTorques(std::vector<Vector> &torques_){
      torques = torques_;
    }
    const std::vector<Vector>& GetTorques(){
      return torques;
    }

    Config GetInitConfiguration(){
      if(q.empty()){
        std::cout << "[PlannerOutput] No path in output, cannot get initial configuration" << std::endl;
        exit(0);
      }
      return q.at(0);
    }

    const SerializedTree& GetTree()
    {
      return _stree;
    }
    void SetTree(SerializedTree &stree)
    {
      _stree = stree;
    }

    const std::vector<Config> GetKeyframes(){
      return q;
    }
    void SetKeyframes(std::vector<Config> &keyframes){
      q = keyframes;
    }

};

