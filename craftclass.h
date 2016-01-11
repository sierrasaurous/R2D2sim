//
//  craftclass.h
//  R2D2ae
//
//  Created by Sierra Adibi on 11/23/15.
//  Copyright © 2015 Sierra Adibi. All rights reserved.
//



#ifndef craftclass_h
#define craftclass_h

using namespace std;



//declare linear degrees of freedom parameters
class linDOF{
public:
    double s, sdot, sdotdot;
    double target;
    
    void initialize();
};

//declare initializations of degree of freedom parameters
void linDOF::initialize(){
    //s = rand()%100;
    //sdot = rand()%5;
    s = 100;
    sdot = 0;
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
    q = -2*atan(1); //Initializing orientation between 0 and 60 degrees
    //q = 0;
    //q = 0.0001;
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
    inertia = 200;
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
    //frame.at(0).s = 50;
    //frame.at(0).sdot = 219.3548;
    frame.at(1).s = 1000000;
    
    //frame.at(0).sdot = frame.at(0).sdot*-1;
}

class State{
public:
    double time, timestep, xpos, xvel;
    double zpos, zvel, phi, phivel, KEx, KEz, KEp;
    
    void get_state(craft l, double t, double ts);
    void printheader();
    void printround(ofstream & file);
    
};

void State::get_state(craft l, double t, double ts){
    time = t;
    timestep = ts;
    xpos = l.frame.at(0).s;
    xvel = l.frame.at(0).sdot;
    zpos = l.frame.at(1).s;
    zvel = l.frame.at(1).sdot;
    phi = l.orientation.at(0).q;
    phivel = l.orientation.at(0).qdot;
    
    KEx = 0.5*pow(xvel,2)*l.mass;
    KEz = 0.5*pow(zvel,2)*l.mass;
    KEp = 0.5*pow(phivel,2)*l.inertia;
}



//print header for keeping track of positions
void State::printheader(){
    cout << "Time \t TStep \t\t X-Pos \t X-Velocity \t Z-Pos \t Z-Velocity \t Pitch \t Omega \t Kinetic Energy" << endl;
}

//print values for each round
//file is the output file to be read by MatLab
void State::printround(ofstream & file){
    
    //cout << time << "\t\t\t" << timestep << "\t\t\t" << xpos << "\t\t\t" << xvel << "\t\t\t" << zpos << "\t\t\t";
    //cout << zvel<< endl; //<< "\t\t\t" << phi << "\t\t\t" << phivel << "\t\t\t" << KEx << "\t\t\t" << KEz << "\t\t\t" << KEp << endl;
    file << fixed << setprecision(3) << time << "\t\t" << timestep << "\t\t" << xpos << "\t\t" << xvel << "\t\t";
    file << fixed << setprecision(9) << zpos << "\t\t";
    file << fixed << setprecision(8) << zvel << "\t\t" << phi << "\t\t" << phivel << "\t\t" << KEx << "\t\t" << KEz << "\t\t" << KEp << endl;
    
}

#endif /* craftclass_h */

