//
// Created by Pavlo Bashmakov on 8/3/17.
//

#ifndef PATH_PLANNING_CONSTANTS_H
#define PATH_PLANNING_CONSTANTS_H

// Activates local execution with graphs (need matplotlib in include)
int LOCAL = 0;


#define PATH_TIMESTEP 0.02  // 50 Hz - rate for final path
#define TRAJ_TIMESTEP 0.2   // 5 Hz - rate for calculating trajectory

#define CAR_LENGTH 5
#define CAR_WIDTH 2.7

#define LANE_WIDTH 4.0

#define SPEED_LIMIT 21.0 // 22.12 // m/s = 49.5 * 1609.344 / 3600

#define MAX_POINTS 50

double METERS_PER_MILE = 1609.344;



#endif //PATH_PLANNING_CONSTANTS_H
