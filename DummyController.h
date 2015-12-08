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
    double linforce;
    double theta, alpha;
    
    forcex = 0.5;
    forcez = 0.5;
    angle = atan(abs(forcez/forcex));
    torque = 0;
    
    double velocity = sqrt(pow(state.xvel,2)+pow(state.zvel,2));
    theta = acos(state.xvel/(velocity+1));
    alpha = state.phi-theta;
    
//    if(alpha <0){
//        torque = .1;
//    }else if(alpha >= 0 && alpha <= 0.1*atan(1)){
//        torque = 0.05;
//    }else{
//        torque = -0.05;
//    }
    
    //double linforce = sqrt(pow(forcex,2)+pow(forcez,2));
    
    linforce = 0;
    
    if(state.time<2){
        linforce = 0;
    }else if(state.time>=2 && state.time<60){
        linforce = 120;
    }else if(state.time>=80){
        if(state.zvel<=-0.5){
            linforce = 20.85;
        }
    }
    
    
    controls.push_back(linforce);
    controls.push_back(torque);
    
    return controls;
}



#endif /* DummyController_h */
