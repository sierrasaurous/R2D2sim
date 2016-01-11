//
//  fitness.h
//  R2D2ae
//
//  Created by Sierra Adibi on 11/23/15.
//  Copyright © 2015 Sierra Adibi. All rights reserved.
//

#ifndef fitness_h
#define fitness_h

using namespace std;

double fitnesscalc(double KEini, double targetangle, State endstate){
    double fitval;
    double KEfit;
    double orientationfit;
    
    KEfit = -(endstate.KEz/KEini);
    orientationfit = (targetangle-endstate.phi)/targetangle;
    
    fitval = 0.8*KEfit+0.2*orientationfit;
    
    return fitval;
}

#endif /* fitness_h */
