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
    double target, thrust, gravity;
    
    void initialize();
};

//declare initializations of degree of freedom parameters
void DOF::initialize(){
    s = rand()%100;
    sdot = rand()%10;
    sdotdot = rand()%10;
    target = 0;
    thrust = 0;
    gravity = 0;
    
    cout << s<<"\t\t"<<sdot<<"\t\t"<<endl;
}

//declare a craft with a reference frame and values for mass and moment of inertia
class craft{
public:
    vector<DOF> frame;
    double mass, inertia, KE;
    
    void initialize(int c);
};

//declare initializations of craft parameters, including DOFs
void craft::initialize(int a){
    mass = 10;
    inertia = 20;
    KE = 0;
    
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
double resetangle(double angle){
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
//returning a vector of doubles, where the 0 value is time, the 1 value is the timestep, and then 1+(n*1),1+(n*2),1+(n*3) are position, velocity, and acceleration, respectively, where n ranges from 1 to the total number of DOFs.
void statevector(int count, vector<double> & state, vector<DOF> ref, double t, double tstep){
    
    state.clear();
    state.push_back(t);
    state.push_back(tstep);
    for(int i=0;i<count;i++){
        double s;
        if(i==2){
            s = resetangle(ref.at(i).s);
        }else{
            s = ref.at(i).s;
        }
        double v = ref.at(i).sdot;
        double a = ref.at(i).sdotdot;
        state.push_back(s);
        state.push_back(v);
        state.push_back(a);
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
        cout << "\t\t" << state.at(i*3+2);
        file << "\t\t" << state.at(i*3+2);
    }
    cout<<endl;
    file<<endl;
}


// dummy controller, taking in vector of doubles for state, returning vector of doubles for controls
vector <double> controller(vector<double> state){
    vector<double> controls;
    
    for(int i=0; i<3;i++){
        double thrust = (state.at(i*3+1))*pow(state.at(1),2);
        controls.push_back(thrust);
    }
    
    return controls;
}

// calculate new position, velocity, and acceleration for each direction.
void dynamicscalc(vector<DOF> & ref, vector<double> controls, double m, double I, double ts){
    double alpha = resetangle(ref.at(2).s);
    double g = -9.81;
    ref.at(0).gravity = g*sin(alpha);
    ref.at(1).gravity = g*cos(alpha);
    for(int i=0;i<controls.size();i++){
        
        ref.at(i).thrust = controls.at(i);
        //cout<< i<<"\t\t"<<ref.at(i).thrust<<endl;
        double accelprev = ref.at(i).sdotdot;
        double velprev = ref.at(i).sdotdot;
        
        if(i<=1){
            ref.at(i).sdotdot = ref.at(i).gravity-ref.at(i).thrust/m;
        }else if(i>1){
            ref.at(i).sdotdot = ref.at(i).gravity-ref.at(i).thrust/I;
        }
        
        ref.at(i).sdot = ref.at(i).sdot + 0.5*ts*(accelprev+ref.at(i).sdotdot);
        ref.at(i).s = ref.at(i).s + 0.5*ts*(velprev+ref.at(i).sdot);
    }
}




int main(){
    
    srand(time(NULL));
    int DOF = 3; // 2 linear, 1 rotational
    double t = 0;
    double const tstep = 0.1;
    double const tmax = 60;
    vector<double> state;
    vector<double> controls;
    
    ofstream myfile;
    myfile.open("R2D2data.txt");
    
    craft lander;
    lander.initialize(DOF);
    
    printheader();
    statevector(DOF, state, lander.frame, t, tstep);
    printround(state, DOF, myfile);
    
    while(t<tmax && lander.frame.at(1).s > lander.frame.at(1).target){
        controls = controller(state);
        dynamicscalc(lander.frame, controls, lander.mass, lander.inertia, tstep);
        t = t+tstep;
        statevector(DOF, state, lander.frame, t, tstep);
        printround(state, DOF, myfile);
    }
    
    myfile.close();
    
    
    return 0;
}