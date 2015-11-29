//
//  simulator.h
//  R2D2ae
//
//  Created by Sierra Adibi on 11/28/15.
//  Copyright © 2015 Sierra Adibi. All rights reserved.
//



#ifndef simulator_h
#define simulator_h

#include "craftclass.h"
#include <vector>

using namespace std;


class Simulator{
    
public:
    int linear, rotational, numalf;
    double t, anglechange, fitness;
    double const tstep = 0.1;
    double const tmax = 60;
    double const rhoair = 1.2;
    
    vector<State> stateholder;
    vector<double> controls;
    vector<double> forces;
    vector<vector<double> > aero;
    
    craft lander;
    State currentstate;
    ofstream myfile;
    
    void initialize_sim();
    void run_sim();
    void end_sim();
};

void Simulator::initialize_sim(){
    linear = 2;
    rotational = 1;
    t = 0;
    anglechange = 0;
    myfile.open("SimulatorData.txt");
    numalf = loadaero(aero);
    
    lander.initialize(linear, rotational);
    currentstate.printheader();
    currentstate.get_state(lander, t, tstep);
    currentstate.printround(myfile);
    stateholder.push_back(currentstate);
    
}

void Simulator::run_sim(){
    
    while(t<tmax && lander.frame.at(1).s > lander.frame.at(1).target){
        controls = controller(currentstate);
        forces = forcecalc(controls, lander, rhoair, aero);
        anglechange = anglechange + dynamicscalc(lander, forces, tstep, linear, rotational);
        t = t+tstep;
        
        currentstate.get_state(lander, t, tstep);
        stateholder.push_back(currentstate);
        currentstate.printround(myfile);
    }
    
}

void Simulator::end_sim(){
    fitness = fitnesscalc(lander.KEinitial, lander.orientation.at(0).target, currentstate);
    
    cout << "Fitness: \t" << fitness << "\t Angle Change: \t"<< anglechange << endl;
    
    myfile.close();
    
}


#endif /* simulator_h */
