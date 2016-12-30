//#include <Interface/SimulationGUI.h>
#include <Interface/SimTestGUI.h>
//#include <Interface/QSimTestGUI.h>
//#include <KlamptQt/qtsimtestgui.h>
//#include "pickandplace.h"
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <stdio.h>
#include "src/gui.h"
#include "src/info.h"


int main(int argc,const char** argv) {
        RobotWorld world;

        ForceFieldBackend backend(&world);
        //SimTestBackend backend(&world);
        WorldSimulation& sim=backend.sim;

        //backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hubo_fractal_3.xml");
        backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/athlete_fractal_1.xml");

        Info info(&world);

        info.print();

        GLUISimTestGUI gui(&backend,&world);
        //ForceFieldGUI gui(&backend,&world);
        gui.SetWindowTitle("SimTest2");
        gui.Run();

        // while(1) {
        //         //sim.Advance(dt);
        //         //sim.UpdateModel();
        //         cout<<sim.time<<'\t'<<world.robots[0]->q<<endl;
        // }
        ////run the simulation
        //while(sim.time < 5) {
        //        backend.RenderWorld();
        //        if(sim.time >= 2.0 && sim.time-dt < 2.0) {
        //                Config q;
        //                sim.robotControllers[0]->GetCommandedConfig(q);
        //                q[7] -= 1.0;
        //                //LexicalCast is needed to convert config to string
        //                sim.robotControllers[0]->SendCommand("set_q",LexicalCast(q));
        //                //then move link 7 (hip pitch) 1.5 radians down
        //                q[7] += 1.5;
        //                sim.robotControllers[0]->SendCommand("append_q",LexicalCast(q));
        //        }
        //        sim.Advance(dt);
        //        sim.UpdateModel();
        //        cout<<sim.time<<'\t'<<world.robots[0]->q<<endl;
        //        backend.DoStateLogging_LinearPath(0,"test_state.path");
        //}
        return 0;
}

