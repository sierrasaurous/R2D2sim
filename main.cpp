//
//  main2.cpp
//  R2D2sim
//
//  Created by Sierra Adibi on 10/13/15.
//  Copyright © 2015 Sierra Adibi. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include "craftclass.h"
#include "anglemath.h"
#include "dynamics.h"
#include "communications.h"
#include "fitness.h"
#include "DummyController.h"


using namespace std;





int main(){
    
    
    //-----------------------------------------------------------
    //Initializing values for the simulator
    
    srand(time(NULL));
    int linear = 2;                 // 2 linear degrees of freedom
    int rotational = 1;             // 1 rotational degree of freedom
    double t = 0;                   // time starts at 0
    double const tstep = 0.1;       // timestep is 0.1 second
    double const tmax = 60;         // the simulation can only run for 60 seconds
    double anglechange = 0;         // tracking the total angle change throughout the simulation
    double const rhoair = 1.2;      // density of air for aerodynamic force calculations
    double fitness;                 // creating variable for later use
    //vector<vector<double> > state;
    vector<State> stateholder;     // vector of states
    vector<double> controls;
    vector<double> forces;
    vector<vector<double> > aero;
    int numalf;
    
    // create data output file
    ofstream myfile;
    myfile.open("R2D2data.txt");
    
    numalf = loadaero(aero);
    
////    for(int i=0;i<numalf;i++){
////        for(int j=0;j<3;j++){
////            cout << aero.at(i).at(j) << "\t";
////        }
////        cout << endl;
////    }
    
    craft lander;
    lander.initialize(linear, rotational);
    
    lander.frame.at(0).sdot = -1*lander.frame.at(0).sdot; // force velocity in the x to be negative
    
    State currentstate;
    currentstate.printheader();
    currentstate.get_state(lander, t, tstep);
    currentstate.printround(myfile);
    stateholder.push_back(currentstate);
    
    
    while(t<tmax && lander.frame.at(1).s > lander.frame.at(1).target){
        controls = controller(currentstate);
        forces = forcecalc(controls, lander, rhoair, aero);
        anglechange = anglechange + dynamicscalc(lander, forces, tstep, linear, rotational);
        t = t+tstep;
        
        currentstate.get_state(lander, t, tstep);
        stateholder.push_back(currentstate);
        currentstate.printround(myfile);
    }
    
    //calculate fitness value
    
    fitness = fitnesscalc(lander.KEinitial, lander.orientation.at(0).target, currentstate);
    
    cout << "Fitness: \t" << fitness << "\t Angle Change: \t"<< anglechange << endl;
    
    myfile.close();
    
    
    return 0;
}