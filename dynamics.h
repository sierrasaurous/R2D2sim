//
//  dynamics.h
//  R2D2ae
//
//  Created by Sierra Adibi on 11/23/15.
//  Copyright © 2015 Sierra Adibi. All rights reserved.
//

#include "anglemath.h"
using namespace std;

#ifndef dynamics_h
#define dynamics_h

// calc controls to forces in newtonian directions
// theta describes the angle of the velocity with respect to the x-axis, negative or positive
// phi describes the angle of the body with respect to the negative x-axis
vector<double> forcecalc(vector<double> controller, craft c, double rho, vector<vector<double> > ae, double time) {
    vector<double> forcevec;                        // vector to be returned
    double phi = c.orientation.at(0).q;             //renaming pitch for ease of use
    double lift, drag, lx, lz, dx, dz, tx, tz;      //newtonian force components
    double g = -9.81;                               //gravitational component
    double velsqr = pow(c.frame.at(0).sdot,2)+pow(c.frame.at(1).sdot,2);    //square of total velocity
    double vel = sqrt(velsqr);                      //total velocity
    double theta;                                   //orientation of velocity vector
    double alpha;
    vector<double> coefficients;
    
    if(vel==0){
        theta = 0;
    }else{
        theta = asin(abs(c.frame.at(1).sdot)/vel);
    }
    
    // initialize coefficients of lift and drag to 0
    double cl = 0;
    double cd = 0;
    
    // test which quadrant velocity vector is in
    vector<int> quad = checkquadrant(c);
    
    // calling functions to find coefficients
    coefficients = coeffcalc(quad, phi, theta);
    
    
    
    alpha = coefficients.at(0);
    

    
    // determine if coefficients of lift and drag are known, assign if possible
    for(int i=0;i<ae.size();i++){
        if(ae.at(i).at(0)==round(alpha*45/atan(1))){
            cl = ae.at(i).at(1);
            cd = ae.at(i).at(2);
            break;
        }
    }
    
    
    
    //cout << "AoA is " << alpha << "\t Cl is " << cl << "\t Cd is " << cd << "\n";
    
    //Calculate forces of lift and drag, will be 0 if cl and cd not available from file.
    lift = cl*rho*velsqr*c.sref*0.5;
    drag = cd*rho*velsqr*c.sref*0.5;
    
    //calculate newtonian components of force, lift, drag, and thrust
    tx = controller.at(0)*coefficients.at(1);
    tz = controller.at(0)*coefficients.at(2);
    lx = lift*coefficients.at(3);
    lz = lift*coefficients.at(4);
    dx = drag*coefficients.at(5);
    dz = drag*coefficients.at(6);
    
    //cout<< time << "\t" << alpha << "\t" << dx << "\t" << dz << endl;
    
    //cout << "AOA: "<< coefficients.at(0) << endl << "CL: " << cl << endl << "LX: " << lx << endl << "LZ: " << lz << endl;
    
    // calculate total forces in x and z, return along with moment.
    double forcesx = lx+dx+tx;
    forcevec.push_back(forcesx);
    double forcesz = lz+dz+tz+g*c.mass;
    forcevec.push_back(forcesz);
    forcevec.push_back(controller.at(1));
    return forcevec;
}


// calculate new position, velocity, and acceleration for each direction.
double dynamicscalc(craft & ref, vector<double> forcevec, double ts, int lincount, int rotcount){
    double prevphi = ref.orientation.at(0).q;
    
    //linear calculations using trapezoidal integration
    
    for(int i=0;i<lincount;i++){
        double accelprev = ref.frame.at(i).sdotdot;
        double velprev = ref.frame.at(i).sdot;
        
        ref.frame.at(i).sdotdot = forcevec.at(i)/ref.mass;
        
        // hardcode strong tailwind, putting vertical velocity to zero.
        
        if(i==0){
            ref.frame.at(i).sdot = 0;
        }else if(i!=0){
            ref.frame.at(i).sdot = ref.frame.at(i).sdot + 0.5*ts*(accelprev+ref.frame.at(i).sdotdot);
        }
        ref.frame.at(i).s = ref.frame.at(i).s + 0.5*ts*(velprev+ref.frame.at(i).sdot);
    }
    
    if(ref.frame.at(1).s<0){
        ref.frame.at(1).s=0;
        ref.frame.at(1).sdot = 0;
        ref.frame.at(1).sdotdot = 0;
    }
    
    //rotational calculations using trapezoidal integration
    
    for(int i=0;i<rotcount;i++){
        double accelprev = ref.orientation.at(i).qdotdot;
        double velprev = ref.orientation.at(i).qdot;
        
        ref.orientation.at(i).qdotdot = forcevec.at(i+lincount)/ref.inertia;
        
        ref.orientation.at(i).qdot = ref.orientation.at(i).qdot + 0.5*ts*(accelprev+ref.orientation.at(i).qdotdot);
        ref.orientation.at(i).q = ref.orientation.at(i).q + 0.5*ts*(velprev+ref.orientation.at(i).qdot);
    }
    
    
    
     // return how much the craft rotated
    return ref.orientation.at(0).q - prevphi;
}

#endif /* dynamics_h */
