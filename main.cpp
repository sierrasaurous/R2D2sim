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
#include "craftclass.h"
#include "anglemath.h"
#include "dynamics.h"
#include "communications.h"
#include "fitness.h"
#include "DummyController.h"
#include "simulator.h"


using namespace std;





int main(){
    
    
    srand(time(NULL));
    
    Simulator sim;
    
    sim.initialize_sim();
    sim.run_sim();
    sim.end_sim();
    
    return 0;
}