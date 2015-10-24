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

using namespace std;

//declare degrees of freedom parameters
class DOF{
public:
    double s, sdot, sdotdot;
    double target;
    
    void initialize();
};

//declare initializations of degree of freedom parameters
void DOF::initialize(){
    s = rand()%100;
    sdot = rand()%10;
    sdotdot = 0;
    target = 0;
    
    cout << s<<"\t\t"<<sdot<<"\t\t"<<endl;
}

//declare a craft with a reference frame and values for mass and moment of inertia
class craft{
public:
    vector<DOF> frame;
    double mass, inertia, KE;
    double CL, CD, sref;
    
    void initialize(int c);
};

//declare initializations of craft parameters, including DOFs
void craft::initialize(int a){
    mass = 10;
    inertia = 20;
    KE = 0;
    CL = 0.9;
    CD = 0.2;
    sref = 100;
    
    for(int i=0;i<a;i++){
        DOF d;
        d.initialize();
        frame.push_back(d);
        if(i<=1){
            KE = KE + 0.5*mass*d.sdot*d.sdot;
        }else if(i>1){
            KE = KE + 0.5*inertia*d.sdot*d.sdot;
        }
    }
}

//make sure the angle is between pi and negative pi
double reset_angle(double angle){
    double pi = 4*atan(1);
    
    if(angle>pi){
        while(angle>pi){
            angle = angle-2*pi;
        }
    }else if(angle< (pi*-1)){
        while(angle<(pi*-1)){
            angle = angle + 2*pi;
        }
    }else if((-1*pi)<=angle && angle <=pi){
        angle = angle;
    }
    return angle;
}



//create a vector of the state variables
//feeding in the number of DOFs, the vector of doubles that will go to controller, the reference frame information, and the time.
//returning a vector of doubles, where the 0 value is time, the 1 value is the timestep, and then the values alternate between position and velocity for the DOFs.
void statevector(int count, vector<double> & state, vector<DOF> ref, double t, double tstep){
    
    state.clear();
    state.push_back(t);
    state.push_back(tstep);
    
    for(int i=0;i<count;i++){
        double s = ref.at(i).s;
        double v = ref.at(i).sdot;
        state.push_back(s);
        state.push_back(v);
    }
}


//print header for keeping track of positions
void printheader(){
    cout << "Time \t X-Pos \t\t Z-Pos \t\t Pitch \t\t Energy" << endl;
}

//print values for each round
//state is the vector of state variables
//count is the number of DOFs
//file is the output file to be read by MatLab
void printround(vector<double> state, int count, ofstream & file){
    cout<<setiosflags(ios::fixed)<<setprecision(1)<<state.at(0);
    file<<setiosflags(ios::fixed)<<setprecision(1)<<state.at(0);
    for(int i=0;i<count;i++){
        cout << "\t\t" << state.at(i*2+2);
        file << "\t\t" << state.at(i*2+2);
    }
    cout<<endl;
    file<<endl;
}


// dummy controller, taking in vector of doubles for state, returning vector of doubles for controls
vector <double> controller(vector<double> state){
    vector<double> controls;
    
    double thrust = (state.at(4)-state.at(5))*state.at(1);
    double moment = (state.at(6)-state.at(7))*state.at(1);
    controls.push_back(thrust);
    controls.push_back(moment);
    
    return controls;
}

// calc controls to forces in newtonian directions
vector<double> forcecalc(vector<double> controls, craft c, double rho) {
    vector<double> forces;
    double alpha = c.frame.at(2).s;
    double theta = atan(c.frame.at(1).sdot/c.frame.at(0).sdot);
    double totalangle = alpha+theta;
    double lift, drag, lx, lz, dx, dz, tx, tz;
    double g = -9.81;
    double velsqr = pow(c.frame.at(0).sdot,2)+pow(c.frame.at(1).sdot,2);
    
    lift = c.CL*rho*velsqr*c.sref*0.5;
    drag =c.CD*rho*velsqr*c.sref*0.5;
    
    lx = -lift*sin(theta);
    lz = lift*cos(theta);
    dx = -drag*cos(theta);
    dz = -drag*sin(theta);
    tx = controls.at(0)*cos(totalangle);
    tz = controls.at(0)*sin(totalangle);
    
    
    double forcesx = lx+dx+tx;
    forces.push_back(forcesx);
    double forcesz = lz+dz+tz+g;
    forces.push_back(forcesz);
    forces.push_back(controls.at(1));
    return forces;
}


// calculate new position, velocity, and acceleration for each direction.
// check trig things
double dynamicscalc(vector<DOF> & ref, vector<double> force, double m, double I, double ts){
    double prevalpha;
    for(int i=0;i<force.size();i++){
        double accelprev = ref.at(i).sdotdot;
        double velprev = ref.at(i).sdot;
        
        if(i<=1){
            ref.at(i).sdotdot = force.at(i)/m;
        }else if(i>1){
            ref.at(i).sdotdot = force.at(i)/I;
        }
        
        ref.at(i).sdot = ref.at(i).sdot + 0.5*ts*(accelprev+ref.at(i).sdotdot);
        ref.at(i).s = ref.at(i).s + 0.5*ts*(velprev+ref.at(i).sdot);
    }
    
    return ref.at(2).s - prevalpha;
}




int main(){
    
    srand(time(NULL));
    int DOF = 3; // 2 linear, 1 rotational
    double t = 0;
    double const tstep = 0.1;
    double const tmax = 60;
    double anglechange = 0;
    double const rhoair = 1.2;
    vector<double> position;
    vector<double> state;
    vector<double> controls;
    vector<double> forces;
    
    ofstream myfile;
    myfile.open("R2D2data.txt");
    
    craft lander;
    lander.initialize(DOF);
    
    lander.frame.at(2).s = reset_angle(lander.frame.at(2).s);
    printheader();
    statevector(DOF, state, lander.frame, t, tstep);
    printround(state, DOF, myfile);
    
    
    
    
    while(t<tmax && lander.frame.at(1).s > lander.frame.at(1).target){
        controls = controller(state);
        forces = forcecalc(controls, lander, rhoair);
        anglechange = anglechange + dynamicscalc(lander.frame, forces, lander.mass, lander.inertia, tstep);
        t = t+tstep;
        statevector(DOF, state, lander.frame, t, tstep);
        printround(state, DOF, myfile);
    }
    
    myfile.close();
    
    
    return 0;
}