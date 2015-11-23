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

using namespace std;



// dummy controller, taking in vector of doubles for state, returning vector of doubles for controls
vector <double> controller(vector<double> state){
    vector<double> controls;
    
    double forcex;
    double forcez;
    double torque;
    double angle;
    
    forcex = -2*(state.at(2)+state.at(3)*10);
    forcez = -2*(state.at(4)+state.at(5)*10);
    angle = atan(abs(forcez/forcex));
    torque = (state.at(6)-angle)/state.at(1);
    
    double linforce = sqrt(pow(forcex,2)+pow(forcez,2));
    
    controls.push_back(linforce);
    controls.push_back(torque);
    
    return controls;
}




int main(){
    
    srand(time(NULL));
    int linear = 2; // 2 linear degrees of freedom
    int rotational = 1; // 1 rotational degree of freedom
    double t = 0;
    double const tstep = 0.1;
    double const tmax = 60;
    double anglechange = 0;
    double const rhoair = 1.2;
    double fitness;
    vector<double> position;
    vector<vector<double> > state;
    vector<double> stateholder;
    vector<double> controls;
    vector<double> forces;
    vector<vector<double> > aero;
    int numalf;
    
    // create data output file
    ofstream myfile;
    myfile.open("R2D2data.txt");
    
    numalf = loadaero(aero);
    
    for(int i=0;i<numalf;i++){
        for(int j=0;j<3;j++){
            cout << aero.at(i).at(j) << "\t";
        }
        cout << endl;
    }
    
    craft lander;
    lander.initialize(linear, rotational);
    
    lander.frame.at(0).sdot = -1*lander.frame.at(0).sdot; // force velocity in the x to be negative
    
    printheader();
    stateholder = createstatevector(linear, rotational, lander, t, tstep);
    printround(stateholder, myfile);
    state.push_back(stateholder);
    
    
    while(t<tmax && lander.frame.at(1).s > lander.frame.at(1).target){
        controls = controller(stateholder);
        forces = forcecalc(controls, lander, rhoair, aero);
        anglechange = anglechange + dynamicscalc(lander, forces, tstep, linear, rotational);
        t = t+tstep;
        stateholder = createstatevector(linear, rotational, lander, t, tstep);
        printround(stateholder, myfile);
        state.push_back(stateholder);
    }
    
    //calculate fitness value
    
    fitness = fitnesscalc(lander.KEinitial, lander.orientation.at(0).target, state.back());
    
    cout << "Fitness: \t" << fitness << "\t Angle Change: \t"<< anglechange << endl;
    
    myfile.close();
    
    
    return 0;
}