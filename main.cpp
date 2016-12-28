#include <Interface/SimulationGUI.h>
#include <Interface/SimTestGUI.h>
#include <stdio.h>


int main(int argc,const char** argv) {
        //create a world
        RobotWorld world;
        SimTestBackend backend(&world);
        WorldSimulation& sim=backend.sim;

        //world.LoadElement("/home/aorthey/git/Klampt/data/robots/athlete.rob");
        //world.LoadElement("/home/aorthey/git/Klampt/data/terrains/fractal_terrain_2.tri");

        //backend.InitSim();
        backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/athlete_fractal_1.xml");
        //if(!backend.LoadAndInitSim(argc,argv)) {
                //cerr<<"Error loading simulation from command line"<<endl;
                //return 1;
        //}

        //Uncomment+edit the following line to change the controller
        //time step for robot 0 (100Hz is the default)
        //sim.controlSimulators[0].controlTimeStep = 0.01;

        //Uncomment+edit the following line to change the underlying
        //simulation time step (1kHz is the default)
        //sim.simStep = 0.001;

        double dt = 0.1;

        //backend.Start();
        //backend.RenderWorld();
        //backend.RenderScreen();
        GLUISimTestGUI gui(&backend,&world);
        gui.SetWindowTitle("SimTest");
        gui.Run();

        while(1) {
                sim.Advance(dt);
                sim.UpdateModel();
                cout<<sim.time<<'\t'<<world.robots[0]->q<<endl;
        }
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

