#include <Interface/SimTestGUI.h>
#include <stdio.h>


int main(int argc,const char** argv) {
        RobotWorld world;
        SimTestBackend backend(&world);
        WorldSimulation& sim=backend.sim;

        //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/athlete_fractal_1.xml");
        //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hrp2.xml");
        //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hubo_fractal_2.xml");
        //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hubo_fractal_3.xml");
        //backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/hubo_object.xml");
        backend.LoadAndInitSim("/home/aorthey/git/orthoklampt/data/sentinel.xml");

        GLUISimTestGUI gui(&backend,&world);
        gui.SetWindowTitle("SimTest");
        gui.Run();
        return 0;
}

