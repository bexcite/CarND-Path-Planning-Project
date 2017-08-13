//
// Created by Pavlo Bashmakov on 8/6/17.
//

#ifndef PATH_PLANNING_HELPERS_PLOT_H
#define PATH_PLANNING_HELPERS_PLOT_H

#include <iostream>
#include "matplotlibcpp.h"
#include "helpers.h"
#include "constants.h"

using namespace std;

namespace plt = matplotlibcpp;



void plot_traj(Trajectory traj, double timestep = TRAJ_TIMESTEP) {

//  double timestep = TRAJ_TIMESTEP;

  auto traj_data = getSDbyTraj(traj, timestep);
  vector<double> SS = traj_data[0];
  vector<double> DD = traj_data[1];
  vector<double> TT = traj_data[2];


  vector<double> s_coeffs_v = differentiate(traj.s_coeffs);
  vector<double> s_coeffs_a = differentiate(s_coeffs_v);


  plt::subplot(4, 1, 1);
  plt::title("Trajectory");

  // General settings
  plt::ylim(0, 12);
  plt::grid(true);

  // Show our car
//  plt::plot({car_s}, {car_d}, "ro");

  // Show traj
  plt::plot(SS, DD, "bo");
//  plt::plot(TT, SS, "bo");

  // Annotate traj
  for (int i = 0; i < TT.size(); i += int(1.0/timestep)) {
    ostringstream oss;
    oss << " " << TT[i];
    plt::annotate(oss.str(), SS[i], DD[i] + 0.2);
//    plt::annotate(oss.str(), TT[i], SS[i] + 0.2);
  }


  plt::subplot(4, 1, 2);
  plt::title("Speed (v)");

  vector<double> SS_v;
  vector<double> DD_v;
  for (int i = 0; i < TT.size(); ++i) {
    SS_v.push_back(poly_calc(traj.s_coeffs, TT[i], 1));
    DD_v.push_back(poly_calc(traj.d_coeffs, TT[i], 1));
  }
  plt::plot(TT, SS_v, "bo");

  // Annotate speed
  for (int i = 0; i < TT.size(); i += int(1.0/timestep)) {
    ostringstream oss;
    oss << "  " << TT[i];
    plt::annotate(oss.str(), TT[i], SS_v[i]);
  }


  plt::subplot(4, 1, 3);
  plt::title("Acc (a)");

  // Total acc < 10m/s*s

  vector<double> SS_a;
  vector<double> DD_a;
  for (int i = 0; i < TT.size(); ++i) {
    double a = poly_calc(traj.s_coeffs, TT[i], 2);
    SS_a.push_back(a);
    DD_a.push_back(poly_calc(traj.d_coeffs, TT[i], 2));
  }
  plt::plot(TT, SS_a, "bo");

  // Annotate acc
  for (int i = 0; i < TT.size(); i += int(1.0/timestep)) {
    ostringstream oss;
    oss << "  " << TT[i];
    plt::annotate(oss.str(), TT[i], SS_a[i]);
  }


  plt::subplot(4, 1, 4);
  plt::title("Jerk (jerk)");

  // Total jerk < 10m/s*s*s

  vector<double> SS_j;
  vector<double> DD_j;
  for (int i = 0; i < TT.size(); ++i) {
    double jerk = poly_calc(traj.s_coeffs, TT[i], 3);
    SS_j.push_back(jerk);
    DD_j.push_back(poly_calc(traj.d_coeffs, TT[i], 3));
  }
  plt::plot(TT, SS_j, "bo");

  // Annotate jerk
  for (int i = 0; i < TT.size(); i += int(1.0/timestep)) {
    ostringstream oss;
    oss << "  " << TT[i];
    plt::annotate(oss.str(), TT[i], SS_j[i]);
  }


  plt::show();




}



#endif //PATH_PLANNING_HELPERS_PLOT_H
