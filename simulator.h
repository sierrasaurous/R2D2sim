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
    double const tmax = 1000;
    double const rhoair = 1.2;
    
    vector<State> stateholder;
    vector<double> controls;
    vector<double> forces;
    vector<vector<double> > aero;
    vector<double> xpositions;
    vector<double> zpositions;
    vector<double> xvels;
    vector<double> zvels;
    vector<double> xke;
    vector<double> zke;
    vector<double> xaccels;
    vector<double> zaccels;
    vector<double> anglepos;
    vector<double> anglevel;
    vector<double> angleaccel;
    vector<double> angleke;
    
    craft lander;
    State currentstate;
    ofstream myfile;
    
    void initialize_sim();
    void fitnessvector();
    void run_sim();
    void end_sim();
};

void Simulator::fitnessvector(){
    xpositions.push_back(currentstate.xpos);
    zpositions.push_back(currentstate.zpos);
    xvels.push_back(currentstate.xvel);
    zvels.push_back(currentstate.zvel);
    xke.push_back(currentstate.KEx);
    zke.push_back(currentstate.KEz);
    xaccels.push_back(lander.frame.at(0).sdotdot);
    zaccels.push_back(lander.frame.at(1).sdotdot);
    anglepos.push_back(currentstate.phi);
    anglevel.push_back(currentstate.phivel);
    angleaccel.push_back(lander.orientation.at(0).qdotdot);
    angleke.push_back(currentstate.KEp);
}

void Simulator::initialize_sim(){
    linear = 2;
    rotational = 1;
    t = 0;
    anglechange = 0;
    myfile.open("SimulatorData.txt");
    numalf = loadaero(aero);
    fitness = 0;
    
    stateholder.clear();
    lander.frame.clear();
    lander.orientation.clear();
    xpositions.clear();
    zpositions.clear();
    xvels.clear();
    zvels.clear();
    xke.clear();
    zke.clear();
    xaccels.clear();
    zaccels.clear();
    anglepos.clear();
    anglevel.clear();
    angleaccel.clear();
    angleke.clear();
    
    lander.initialize(linear, rotational);
    currentstate.printheader();
    currentstate.get_state(lander, t, tstep);
    currentstate.printround(myfile);
    stateholder.push_back(currentstate);
    
    fitnessvector();
    
}

void Simulator::run_sim(){
    
    while(t<tmax){// && lander.frame.at(1).s >= lander.frame.at(1).target){
        controls = controller(currentstate);
        forces = forcecalc(controls, lander, rhoair, aero);
        anglechange = anglechange + dynamicscalc(lander, forces, tstep, linear, rotational);
        t = t+tstep;
        
        currentstate.get_state(lander, t, tstep);
        stateholder.push_back(currentstate);
        currentstate.printround(myfile);
        fitnessvector();
    }
    
}

void Simulator::end_sim(){
    fitness = fitnesscalc(lander.KEinitial, lander.orientation.at(0).target, currentstate);
    
    cout << "Fitness: \t" << fitness << "\t Angle Change: \t"<< anglechange << endl;
    myfile.close();
    
}


#endif /* simulator_h */
