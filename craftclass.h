//
//  craftclass.h
//  R2D2ae
//
//  Created by Sierra Adibi on 11/23/15.
//  Copyright © 2015 Sierra Adibi. All rights reserved.
//

using namespace std;

#ifndef craftclass_h
#define craftclass_h

//declare linear degrees of freedom parameters
class linDOF{
public:
    double s, sdot, sdotdot;
    double target;
    
    void initialize();
};

//declare initializations of degree of freedom parameters
void linDOF::initialize(){
    s = rand()%100;
    sdot = rand()%5;
    sdotdot = 0;
    target = 0;
    
    cout << s<<"\t\t"<<sdot<<"\t\t"<<endl;
}

class rotDOF{
public:
    double q, qdot, qdotdot;
    double target;
    
    void initialize();
};


void rotDOF::initialize(){
    q = (rand()%60)*4*atan(1)/180; //Initializing orientation between 0 and 60 degrees
    qdot = 0;
    qdotdot = 0; //Initializing angular velocity and acceleration to 0 for simplicity.
    target = 0;
    
}

//declare a craft with a reference frame and values for mass and moment of inertia
class craft{
public:
    vector<linDOF> frame;
    vector<rotDOF> orientation;
    double mass, inertia, KEinitial;
    double sref;
    
    void initialize(int l, int r);
};

//declare initializations of craft parameters, including DOFs
void craft::initialize(int dl, int dr){
    mass = 20;
    inertia = 20;
    KEinitial = 0;
    sref = 0.02; // wing considered roughly rectangular with 20 cm chord length, 1 m span
    
    for(int i=0;i<dl;i++){
        linDOF lin;
        lin.initialize();
        KEinitial = KEinitial + 0.5*mass*pow(lin.sdot,2);
        frame.push_back(lin);
    }
    
    for(int i=0;i<dr;i++){
        rotDOF rot;
        rot.initialize();
        KEinitial = KEinitial + 0.5*inertia*pow(rot.qdot,2);
        orientation.push_back(rot);
    }
}

#endif /* craftclass_h */

