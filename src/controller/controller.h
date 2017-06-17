#pragma once
#include <Control/Controller.h>
#include <KrisLibrary/math/random.h>

struct ControllerState{
  public:
    std::vector<Vector3> predicted_com;
    std::vector<Vector3> predicted_com_dir;
    std::vector<Vector3> com_window;
    std::vector<Vector3> linmomentum_window;
    std::vector<Vector3> angmomentum_window;

    double mass;
    double maximumWindowLength = 2.0;

    void AddCOM( Vector3 &com, Vector3 &linmom, Vector3 &angmom );
    void SetMass( double _mass );
    void PredictCOM( double tstep, uint Nsteps);

    Vector current_torque;

    void Reset();

  private:
    double getLength(std::vector<Vector3> &path);
};


class ContactStabilityController: public RobotController
{
  protected:
    ControllerState output;
    std::vector<Vector> torques;
    std::vector<double> times;
    double overall_time;
    Vector ZeroTorque;

  public:
    ContactStabilityController(Robot& robot);
    virtual ~ContactStabilityController();
    virtual const char* Type() const;
    virtual void Reset();

    const ControllerState& GetControllerState() const;
    virtual void Update(Real dt);
    virtual bool SendCommand(const string& name,const string& str);
    void AppendTorqueAndTime( Vector &torque_and_time );

    virtual vector<string> Commands() const;

};

