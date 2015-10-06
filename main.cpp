//
//  main.cpp
//  R2D2sim
//
//  Created by Sierra Adibi on 10/5/15.
//  Copyright © 2015 Sierra Adibi. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>

using namespace std;

class linDOF{
public:
    double s, sdot, sdotdot;
    double target, thrust;
    
    void initialize();
};

void linDOF::initialize(){
    s = rand()%90;
    sdot = rand()%10;
    sdotdot = rand()%10;
    
    target = 0;
    thrust = 0;
}

class rotDOF{
public:
    double theta, thetadot, thetadotdot;
    double target, torque;
    
    void initialize();
};

void rotDOF::initialize(){
    theta = rand()%100;
    thetadot = rand()%10;
    thetadotdot = rand()%10;
    
    target = 0;
    torque = 0;
}


class craft{
public:
    vector<linDOF> linfreedom;
    vector<rotDOF> rotfreedom;
    double mass, inertia;
    
    void initialize(int lincount, int rotcount);
    void printinitial();
};

void craft::initialize(int lincount, int rotcount){
    mass = 10;
    inertia = 20;
    
    for(int i=0; i<lincount; i++){
        linDOF l;
        l.initialize();
        linfreedom.push_back(l);
    }
    for(int i=0; i<rotcount; i++){
        rotDOF r;
        r.initialize();
        rotfreedom.push_back(r);
    }
}

void craft::printinitial(){
    for(int i=0;i<linfreedom.size();i++){
        cout<<setiosflags(ios::fixed)<<setprecision(1)<<"Initial "<<i<<" Position: "<<linfreedom.at(i).s;
        cout<<" Target: "<<linfreedom.at(i).target<<endl;
    }
    for(int i=0;i<rotfreedom.size();i++){
        cout<<setiosflags(ios::fixed)<<setprecision(1)<<"Initial "<<i<<" Angle: "<<rotfreedom.at(i).theta;
        cout<<" Target: "<<rotfreedom.at(i).target<<endl;
    }
}

void printheader(){
    cout << "Time \t X-Pos \t\t Z-Pos \t\t Angle \t\t Energy" << endl;
}

void printround(){
    
    
    
}


vector <double> lincontroller(double pos, double vel, double acc, double target, double timestep){
    vector<double> lincontrols;
    
    for(int i=0; i<2;i++){
        double thrust = (pos-target)*pow(timestep,2);
        lincontrols.push_back(thrust);
    }
    
    return lincontrols;
}

double directioncheck(int x){
    double gravity; //For this version, defining x as 0, z as 1
    if( x==1){
        gravity = -9.81;
    }else{
        gravity = 0;
    }
    
    return gravity;
}

double anglecheck(double currentangle){
    double pi = 3.1415926536;
    double negpi = -1*pi;
    
    while(currentangle<negpi){
        currentangle = currentangle + 2*pi;
    }
    while(currentangle>pi){
        currentangle = currentangle + 2*negpi;
    }
    
    return currentangle;
}

void lineardynamics(linDOF & direction, double ts, double gravity, double m){
    double accelprev = direction.sdotdot;
    double velprev = direction.sdot;
    
    //direction.sdotdot = gravity-thrust/m;
    direction.sdot = direction.sdot + 0.5*ts*(accelprev+direction.sdotdot);
    direction.s = direction.s + 0.5*ts*(velprev+direction.sdot);
}

void rotationaldynamics(rotDOF & orientation, double ts, double I){
    double direction = anglecheck(orientation.theta);
    //double torque = rotcontroller(orientation.theta, orientation.thetadot, orientation.thetadotdot);
    double alphaprev = orientation.thetadotdot;
    double omegaprev = orientation.thetadot;
    
    //orientation.thetadotdot = torque/I;
    orientation.thetadot = orientation.thetadot + 0.5*ts*(alphaprev+orientation.thetadotdot);
    orientation.theta = orientation.theta + 0.5*ts*(omegaprev+orientation.thetadot);
}






int main(){
    
    srand(time(NULL));
    int linDOF = 2;
    int rotDOF = 1;
    double t = 0;
    double tstep = 0.1;
    double const tmax = 60;
    double pi = 4*atan(1);
    double g, energy, d;
    
    cout << "Pi is " << pi;
    
    ofstream myfile;
    myfile.open("R2D2data.txt");
    
    craft lander;
    lander.initialize(linDOF, rotDOF);
    lander.printinitial();
    printheader();
    
    while(t<tmax && lander.linfreedom.at(1).s > lander.linfreedom.at(1).target){
        for( int i=0; i<linDOF; i++){
            g = directioncheck(i);
            lineardynamics(lander.linfreedom.at(i), tstep, g, lander.mass);
        }
        for(int i=0; i<rotDOF;i++){
            rotationaldynamics(lander.rotfreedom.at(i), tstep, lander.inertia);
        }
        
        
    }
    
    
    
    
    
    
    
    return 0;
}
