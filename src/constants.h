//
// Created by Pavlo Bashmakov on 8/3/17.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

// Activates local execution with graphs (need matplotlib in include)
int LOCAL = 1;


#define PATH_TIMESTEP 0.02  // 50 Hz - rate for final path
#define TRAJ_TIMESTEP 0.2   // 5 Hz - rate for calculating trajectory

double METERS_PER_MILE = 1609.344;


#endif //PATH_PLANNING_CONSTANTS_H
