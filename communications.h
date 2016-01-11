//
//  communications.h
//  R2D2ae
//
//  Created by Sierra Adibi on 11/23/15.
//  Copyright © 2015 Sierra Adibi. All rights reserved.
//

using namespace std;

#ifndef communications_h
#define communications_h

//create a vector of the state variables
//feeding in the number of DOFs, the vector of doubles that will go to controller, the reference frame information, and the time.
//returning a vector of doubles, where the 0 value is time, the 1 value is the timestep, and then the values alternate between position and velocity for the DOFs. The final value is the Kinetic Energy
//vector<double> createstatevector(int lincount, int rotcount, craft ref, double time, double timestep){
//    
//    vector<double> statevec;
//    double KE = 0;
//    
//    
//    statevec.push_back(time);
//    statevec.push_back(timestep);
//    
//    for(int i=0;i<lincount;i++){
//        double s = ref.frame.at(i).s;
//        double v = ref.frame.at(i).sdot;
//        KE = KE + 0.5*ref.mass*v*v;
//        statevec.push_back(s);
//        statevec.push_back(v);
//    }
//    
//    for(int i=0;i<rotcount;i++){
//        double theta = ref.orientation.at(i).q;
//        double omega = ref.orientation.at(i).qdot;
//        statevec.push_back(theta);
//        statevec.push_back(omega);
//        KE = KE + 0.5*ref.inertia*omega*omega;
//    }
//    
//    statevec.push_back(KE);
//    return statevec;
//}


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


#endif /* communications_h */
