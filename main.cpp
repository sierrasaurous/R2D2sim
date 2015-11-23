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
    
    void initialize();
};


void rotDOF::initialize(){
    q = (rand()%60)*4*atan(1)/180; //Initializing orientation between 0 and 60 degrees
    qdot = 0;
    qdotdot = 0; //Initializing angular velocity and acceleration to 0 for simplicity.
    
}

//declare a craft with a reference frame and values for mass and moment of inertia
class craft{
public:
    vector<linDOF> frame;
    vector<rotDOF> orientation;
    double mass, inertia, KE;
    double sref;
    
    void initialize(int l, int r);
};

//declare initializations of craft parameters, including DOFs
void craft::initialize(int dl, int dr){
    mass = 20;
    inertia = 20;
    KE = 0;
    sref = 0.02; // wing considered roughly rectangular with 20 cm chord length, 1 m span
    
    for(int i=0;i<dl;i++){
        linDOF lin;
        lin.initialize();
        frame.push_back(lin);
    }
    
    for(int i=0;i<dr;i++){
        rotDOF rot;
        rot.initialize();
        orientation.push_back(rot);
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
vector<double> statevector(int lincount, int rotcount, craft ref, double time, double timestep){
    
    vector<double> statevec;
    
    statevec.push_back(time);
    statevec.push_back(timestep);
    
    for(int i=0;i<lincount;i++){
        double s = ref.frame.at(i).s;
        double v = ref.frame.at(i).sdot;
        statevec.push_back(s);
        statevec.push_back(v);
    }
    
    for(int i=0;i<rotcount;i++){
        statevec.push_back(ref.orientation.at(i).q);
        statevec.push_back(ref.orientation.at(i).qdot);
    }
    
    return statevec;
}


//print header for keeping track of positions
void printheader(){
    cout << "Time \t TStep \t\t X-Pos \t X-Velocity \t Z-Pos \t Z-Velocity \t Pitch \t Omega \t Energy" << endl;
}

//print values for each round
//state is the vector of state variables
//count is the number of DOFs
//file is the output file to be read by MatLab
void printround(vector<double> s, ofstream & file){
    for(int i=0;i<s.size();i++){
        cout << s.at(i) << "\t\t\t";
        file << s.at(i) << "\t\t\t";
    }
    cout<<endl;
    file<<endl;

}


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

// check which quadrant the velocity vector and aircraft x vector are in
vector<int> checkquadrant(craft land){
    int direction;
    vector<int> quadrants;
    double phi;
    
    // check quadrant for velocity vector
    if (land.frame.at(0).sdot >= 0){
        if(land.frame.at(1).sdot >= 0){
            direction = 1; //first quadrant
        }else{
            direction = 4; //fourth quadrant
        }
    }else{
        if(land.frame.at(1).sdot >= 0){
            direction = 2; //second quadrant
        }else{
            direction = 3; //third quadrant
        }
    }
    quadrants.push_back(direction);
    
    // check quadrant for aircraft body
    
    phi = reset_angle(land.orientation.at(0).q);
    double pi = 4*atan(1);
    
    if(0<=phi && phi<=(pi/2)){
        direction = 2;
    }else if((pi/2)<phi&& phi <=pi){
        direction = 1;
    }else if(phi<0 && phi>=(-pi/2)){
        direction = 3;
    }else if((-pi/2)>phi&& phi>=(-pi)){
        direction = 4;
    }else{
        cout << "\n ERROR, HOMIE \n";
    }
    quadrants.push_back(direction);
    
    return quadrants;
}


// determine how we are making vectors components
// 0 value is angle of attack
// 1,2 are multipliers for lift in nx and nz, respectively
// 3,4 are multipliers for drag in nx and nz
// 5,6 are multipliers for thrust in nx and nz
vector<double> alphacalc(vector<int> directions, double q, double t){
    double aoa;
    double pi = 4*atan(1);
    double lxm, lzm, dxm, dzm, txm, tzm;
    vector<double> coefficients;
    
    if(directions.at(0)==1){
        if(directions.at(1)==1){
            aoa = pi-q-t;
            txm = cos(pi-q);
            tzm = sin(pi-q);
            lxm = -sin(t);
            lzm = cos(t);
            dxm = -cos(t);
            dzm = -sin(t);
        }else if(directions.at(1)==2){
            aoa = pi-q-t;
            txm = -cos(q);
            tzm = sin(q);
            lxm = sin(t);
            lzm = -cos(t);
            dxm = -cos(t);
            dzm = -sin(t);
        }else if(directions.at(1)==3){
            aoa = pi/2;
            txm = -cos(q);
            tzm = -sin(q);
            lxm = 0;
            lzm = 0;
            dxm = 0;
            dzm = 0;
        }else if(directions.at(1)==4){
            aoa = -(pi-q+t);
            txm = cos(pi-q);
            tzm = -sin(pi-q);
            lxm = -sin(t);
            lzm = cos(t);
            dxm = -cos(t);
            dzm = -sin(t);
        }else{
            cout << "\n ERROR, HOMIE \n";
        }
    }else if(directions.at(0)==2){
        if(directions.at(1)==1){
            aoa = q-t;
            txm = cos(pi-q);
            tzm = sin(pi-q);
            lxm = -cos(t);
            lzm = -sin(t);
            dxm = cos(t);
            dzm = -sin(t);
        }else if(directions.at(1)==2){
            aoa = q-t;
            txm = -cos(q);
            tzm = sin(q);
            lxm = cos(t);
            lzm = sin(t);
            dxm = cos(t);
            dzm = -sin(t);
        }else if(directions.at(1)==3){
            aoa = -(q+t);
            txm = -cos(q);
            tzm = -sin(q);
            lxm = -cos(t);
            lzm = sin(t);
            dxm = cos(t);
            dzm = -sin(t);
        }else if(directions.at(1)==4){
            aoa = pi/2;
            txm = cos(pi-q);
            tzm = -sin(pi-q);
            lxm = 0;
            lzm = 0;
            dxm = 0;
            dzm = 0;
        }else{
            cout << "\n ERROR, HOMIE \n";
        }
    }else if(directions.at(0)==3){
        if(directions.at(1)==1){
            aoa = pi/2;
            txm = cos(pi-q);
            tzm = sin(pi-q);
            lxm = 0;
            lzm = 0;
            dxm = 0;
            dzm = 0;
        }else if(directions.at(1)==2){
            aoa = q+t;
            txm = -cos(q);
            tzm = sin(q);
            lxm = -sin(t);
            lzm = cos(t);
            dxm = cos(t);
            dzm = sin(t);
        }else if(directions.at(1)==3){
            aoa = t-q;
            txm = cos(q);
            tzm = -sin(q);
            lxm = -sin(t);
            lzm = cos(t);
            dxm = cos(t);
            dzm = sin(t);
        }else if(directions.at(1)==4){
            aoa = t-q;
            txm = cos(pi-q);
            tzm = -sin(pi-q);
            lxm = sin(t);
            lzm = -cos(t);
            dxm = cos(t);
            dzm = sin(t);
        }else{
            cout << "\n ERROR, HOMIE \n";
        }
    }else if(directions.at(0)==4){
        if(directions.at(1)==1){
            aoa = pi-q+t;
            txm = cos(pi-q);
            tzm = sin(pi-q);
            lxm = sin(t);
            lzm = cos(t);
            dxm = -cos(t);
            dzm = sin(t);
        }else if(directions.at(1)==2){
            aoa = pi/2;
            txm = -cos(q);
            tzm = sin(q);
            lxm = 0;
            lzm = 0;
            dxm = 0;
            dzm = 0;
        }else if(directions.at(1)==3){
            aoa = pi-q-t;
            txm = -cos(q);
            tzm = -sin(q);
            lxm = sin(t);
            lzm = cos(t);
            dxm = -cos(t);
            dzm = sin(t);
        }else if(directions.at(1)==4){
            aoa = pi-q-t;
            txm = cos(pi-q);
            tzm = -sin(pi-q);
            lxm = sin(t);
            lzm = cos(t);
            dxm = -cos(t);
            dzm = sin(t);
        }else{
            cout << "\n ERROR, HOMIE \n";
        }
    }else{
        cout << "\n ERROR, HOMIE \n";
    }
    
    // pushback values into a vector
    coefficients.push_back(aoa);
    coefficients.push_back(txm);
    coefficients.push_back(tzm);
    coefficients.push_back(lxm);
    coefficients.push_back(lzm);
    coefficients.push_back(dxm);
    coefficients.push_back(dzm);
    
    return coefficients;
}



// calc controls to forces in newtonian directions
// theta describes the angle of the velocity with respect to the x-axis
// phi describes the angle of the body with respect to the x-axis
vector<double> forcecalc(vector<double> controller, craft c, double rho, vector<vector<double> > ae) {
    vector<double> forcevec;
    double phi = c.orientation.at(0).q;
    double lift, drag, lx, lz, dx, dz, tx, tz;
    double g = -9.81;
    double velsqr = pow(c.frame.at(0).sdot,2)+pow(c.frame.at(1).sdot,2);
    double vel = sqrt(velsqr);
    double theta = asin(abs(c.frame.at(1).sdot)/vel);
    double cl = 0;
    double cd = 0;
    
    // test which quadrant velocity vector is in
    vector<int> quad = checkquadrant(c);
    
    vector<double> angles = alphacalc(quad, phi, theta);
    
    double alpha = angles.at(0);

    
    // find coefficients of lift and drag
    for(int i=0;i<ae.size();i++){
        if(ae.at(i).at(0)==alpha){
            cl = ae.at(i).at(1);
            cd = ae.at(i).at(2);
            break;
        }
    }
    
    //cout << "AoA is " << alpha << "\t Cl is " << cl << "\t Cd is " << cd << "\n";
    
    lift = cl*rho*velsqr*c.sref*0.5;
    drag = cd*rho*velsqr*c.sref*0.5;
    
    lx = lift*sin(theta);
    lz = lift*cos(theta);
    dx = drag*cos(theta);
    dz = -drag*sin(theta);
    tx = controller.at(0)*cos(phi);
    tz = controller.at(0)*sin(phi);
    
    
    double forcesx = lx+dx+tx;
    forcevec.push_back(forcesx);
    double forcesz = lz+dz+tz+g;
    forcevec.push_back(forcesz);
    forcevec.push_back(controller.at(1));
    return forcevec;
}


// calculate new position, velocity, and acceleration for each direction.
// check trig things
double dynamicscalc(craft & ref, vector<double> forcevec, double ts, int lincount, int rotcount){
    double prevphi = ref.orientation.at(0).q;
    
    //linear calculations
    
    for(int i=0;i<lincount;i++){
        double accelprev = ref.frame.at(i).sdotdot;
        double velprev = ref.frame.at(i).sdot;

        ref.frame.at(i).sdotdot = forcevec.at(i)/ref.mass;
        
        ref.frame.at(i).sdot = ref.frame.at(i).sdot + 0.5*ts*(accelprev+ref.frame.at(i).sdotdot);
        ref.frame.at(i).s = ref.frame.at(i).s + 0.5*ts*(velprev+ref.frame.at(i).sdot);
    }
    
    //rotational calculations
    
    for(int i=0;i<rotcount;i++){
        double accelprev = ref.orientation.at(i).qdotdot;
        double velprev = ref.orientation.at(i).qdot;
        
        ref.orientation.at(i).qdotdot = forcevec.at(i+lincount)/ref.inertia;
        
        ref.orientation.at(i).qdot = ref.orientation.at(i).qdot + 0.5*ts*(accelprev+ref.orientation.at(i).qdotdot);
        ref.orientation.at(i).q = ref.orientation.at(i).q + 0.5*ts*(velprev+ref.orientation.at(i).qdot);
    }
    
    return ref.orientation.at(0).q - prevphi;
}


// Input Aerodynamic Coefficients from .txt file

int loadaero(vector< vector< double> > & a){
    ifstream co("aerocoeff.txt");
    
    double read;
    vector<double> apush;
    int counter = 0;
    
    while(co >> read){
        apush.push_back(read);
        
        if(apush.size()>=3){
            a.push_back(apush);
            apush.clear();
            counter++;
        }
    }
    return counter;
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
    stateholder = statevector(linear, rotational, lander, t, tstep);
    printround(stateholder, myfile);
    state.push_back(stateholder);
    
    
    while(t<tmax && lander.frame.at(1).s > lander.frame.at(1).target){
        controls = controller(stateholder);
        forces = forcecalc(controls, lander, rhoair, aero);
        anglechange = anglechange + dynamicscalc(lander, forces, tstep, linear, rotational);
        t = t+tstep;
        stateholder = statevector(linear, rotational, lander, t, tstep);
        printround(stateholder, myfile);
        state.push_back(stateholder);
    }
    
    cout << anglechange << endl;
    
    myfile.close();
    
    
    return 0;
}