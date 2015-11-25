//
//  DummyController.h
//  R2D2ae
//
//  Created by Sierra Adibi on 11/24/15.
//  Copyright © 2015 Sierra Adibi. All rights reserved.
//

#ifndef DummyController_h
#define DummyController_h

using namespace std;

// dummy controller, taking in vector of doubles for state, returning vector of doubles for controls
vector <double> controller(State state){
    vector<double> controls;
    
    double forcex;
    double forcez;
    double torque;
    double angle;
    
//    forcex = -2*(state.at(2)+state.at(3)*10);
//    forcez = -2*(state.at(4)+state.at(5)*10);
    
    forcex = 0.5;
    forcez = 0.5;
    angle = atan(abs(forcez/forcex));
    torque = (state.phi-angle)/state.timestep;
    
    double linforce = sqrt(pow(forcex,2)+pow(forcez,2));
    
    controls.push_back(linforce);
    controls.push_back(torque);
    
    return controls;
}



#endif /* DummyController_h */
