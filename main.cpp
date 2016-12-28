//#include <Interface/SimulationGUI.h>
#include <Interface/SimTestGUI.h>
//#include <Interface/QSimTestGUI.h>
//#include <KlamptQt/qtsimtestgui.h>
//#include "pickandplace.h"
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/drawextra.h>

#include <stdio.h>

// TODO: draw arrows here not in SimTestGUI.cpp!
//class SimGUI2 : public SimTestBackend
//{
//        virtual bool OnCommand(const string& cmd,const string& args){
//                if(cmd=="advance") {
//                        glBegin(GL_LINES);
//                        //glDisable(GL_LIGHTING);
//                        //glColor3f(1,0,0);
//                        //glLineWidth(5.0);
//                        glVertex3f(0.0f, 0.0f, 0.0f);
//                        glVertex3f(50.0f, 50.0f, 50.0f);
//                        glEnd();
//                        SimStep(sim.simStep);
//                }
//        }
//
//}

int main(int argc,const char** argv) {
        RobotWorld world;
        SimTestBackend backend(&world);
        WorldSimulation& sim=backend.sim;

        backend.LoadAndInitSim("/home/aorthey/git/Klampt/data/hubo_fractal_3.xml");

        int ids = world.NumIDs();

        std::cout << std::string(80, '-') << std::endl;

        std::cout << "RobotWorld Info" << std::endl;

        std::cout << std::string(80, '-') << std::endl;
        for(int itr = 0; itr <= ids; itr++){
                std::cout << "[" << itr << "] " << world.GetName(itr) << std::endl;
        }
        std::cout << std::string(80, '-') << std::endl;

        std::vector<SmartPointer<Robot> > robots = world.robots;
        std::vector<SmartPointer<Terrain> > terrains = world.terrains;
        for (std::vector<SmartPointer<Robot> >::iterator it = robots.begin() ; it != robots.end(); ++it){
                std::cout << "Robot " << (*it)->name << std::endl;
        }

        GLUISimTestGUI gui(&backend,&world);
        gui.SetWindowTitle("SimTest2");
        glBegin(GL_LINES);
        //glDisable(GL_LIGHTING);
        //glColor3f(1,0,0);
        //glLineWidth(5.0);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(50.0f, 50.0f, 50.0f);
        glEnd();
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

