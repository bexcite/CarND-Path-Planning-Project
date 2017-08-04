//
// Created by Pavlo Bashmakov on 8/3/17.
//

#ifndef PATH_PLANNING_TEST_LOCAL_H
#define PATH_PLANNING_TEST_LOCAL_H


#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
//#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "json.hpp"
#include "matplotlibcpp.h"
#include <typeinfo>
#include <unistd.h>
#include "spline.h"
#include "helpers.h"

using namespace std;


// for convenience
using json = nlohmann::json;


namespace plt = matplotlibcpp;

// Test on local data
void testLocal(std::string s, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {
  json j = json::parse(s);

  cout << endl << ">>> TESTING on STATIC DATA !!!!!" << endl << endl;

  cout << "testJdata = " << j << endl;

  // Main car's localization Data
  double car_x = j["x"];
  double car_y = j["y"];
  double car_s = j["s"];
  double car_d = j["d"];
  double car_yaw = j["yaw"];
  double car_speed = j["speed"];

  // Previous path data given to the Planner
  auto previous_path_x = j["previous_path_x"];
  auto previous_path_y = j["previous_path_y"];
  // Previous path's end s and d values
  double end_path_s = j["end_path_s"];
  double end_path_d = j["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = j["sensor_fusion"];

  cout << "decltype = " << typeid(sensor_fusion).name() << endl;

  vector<double> S;
  vector<double> D;
  vector<int> ID;


  // JMT track

  // Start s_start, s_start_dot, s_start_dot_dot

//  double s_start = car_s;
//  double s_start_dot = car_speed;
//  double s_start_dot_dot = 0;

  vector<double> s_start = {145.0, car_speed, 0.0}; // {car_s, car_speed, 0};
  vector<double> s_end = {155.0, 3.0, 0.0}; // {car_s+100, 15, 0};

//  vector<double> s_start = {car_s, car_speed, 0.0}; // {car_s, car_speed, 0};
//  vector<double> s_end = {car_s+100, 3.0, 0.0}; // {car_s+100, 15, 0};

  vector<double> d_start = {car_d, 0.0, 0.0};
  vector<double> d_end = {car_d, 0.0, 0.0};

  double T = 4.0;

  auto s_coeffs = JMT(s_start, s_end, T);
  auto d_coeffs = JMT(d_start, d_end, T);

  print_coeffs("s_coeffs : ", s_coeffs);
  print_coeffs("d_coeffs : ", d_coeffs);

  print_coeffs("s_coeffs d1 : ", differentiate(s_coeffs));
  print_coeffs("d_coeffs d1 : ", differentiate(d_coeffs));


  auto clk = chrono::high_resolution_clock::now();
  double dt;

  clk = chrono::high_resolution_clock::now();

  double timestep = 0.1;
  double t = 0.0;
  vector<double> SS;
  vector<double> DD;
  vector<double> TT;
  while (t <= T+timestep) {
    double sx = poly_calc(s_coeffs, t);
    double dx = poly_calc(d_coeffs, t);
    SS.push_back(sx);
    DD.push_back(dx);
    TT.push_back(t);
    t += timestep;
  }

  dt = chrono::duration<double>(chrono::high_resolution_clock::now() - clk).count();
  cout << "DT poly_calc = " << dt << endl;

//  cout << "s[0] = " << poly_calc(s_coeffs, 0.0) << endl;

  clk = chrono::high_resolution_clock::now();

  vector<double> XX;
  vector<double> YY;
  vector<double> XX1;
  vector<double> YY1;
  vector<double> XX2;
  vector<double> YY2;
  auto xy = getXY(SS, DD, maps_s, maps_x, maps_y);
  XX = xy[0];
  YY = xy[1];

  vector<double> DD2;
  vector<double> DD10;
  for (int i = 0; i < SS.size(); ++i) {
    DD2.push_back(2.0);
    DD10.push_back(10.0);
  }

  xy = getXY(SS, DD, maps_s, maps_x, maps_y, false);
  XX1 = xy[0];
  YY1 = xy[1];

//  xy = getXY(SS, DD10, maps_s, maps_x, maps_y);
//  XX2 = xy[0];
//  YY2 = xy[1];

  dt = chrono::duration<double>(chrono::high_resolution_clock::now() - clk).count();
  cout << "DT getXY = " << dt << endl;


  clk = chrono::high_resolution_clock::now();

  // Smoothing XY line
  tk::spline splX;
  splX.set_points(TT, XX);

  tk::spline splY;
  splY.set_points(TT, YY);

  vector<double> XX_smooth;
  vector<double> YY_smooth;
  t = 0.0;
  double ts = 0.02;
  while (t <= T) {
    XX_smooth.push_back(splX(t));
    YY_smooth.push_back(splY(t));
    t += ts;
  }

  dt = chrono::duration<double>(chrono::high_resolution_clock::now() - clk).count();
  cout << "DT smoothing = " << dt << endl;

  plt::subplot(2, 1, 1);
  plt::title("XY and XY_smooth");
  plt::plot(XX, YY, "bo");
  plt::plot(XX1, YY1, "rx");
  plt::plot(XX2, YY2, "bo");
  plt::subplot(2, 1, 2);
  plt::plot(XX_smooth, YY_smooth, "ro");
  plt::show();

  // End s_end, s_end_dot, s_end_dot_dot


  cout << "sensor_fusion:" << endl;
  for (int i = 0; i < sensor_fusion.size(); ++i) {
//    cout << "[" << sensor_fusion[i][0] << "] = "
//         << sensor_fusion[i][5] << ", " << sensor_fusion[i][6] << endl;
    ID.push_back(sensor_fusion[i][0]);
    S.push_back(sensor_fusion[i][5]);
    D.push_back(sensor_fusion[i][6]);
  }


  /*
  // Show other cars
  plt::title("Predictions");
  plt::plot(S, D, "bo");

  // Annotate
  for (int i = 0; i < ID.size(); ++i) {
    ostringstream oss;
    oss << "  " << ID[i];
    plt::annotate(oss.str(), S[i], D[i] + 0.2);
  }
  */

  plt::subplot(4, 1, 1);
  plt::title("Trajectory");

  // General settings
  plt::ylim(0, 12);
  plt::grid(true);

  // Show our car
//  plt::plot({car_s}, {car_d}, "ro");

  // Show traj
  plt::plot(SS, DD, "bo");

  // Annotate traj
  for (int i = 0; i < TT.size(); i += int(1.0/timestep)) {
    ostringstream oss;
    oss << " " << i; // TT[i]
    plt::annotate(oss.str(), SS[i], DD[i] + 0.2);
  }

  for (int i = 0; i < SS.size(); ++i) {
    // Perform
    double s = SS[i];
    double d = DD[i];
    int prev_wp = -1;
    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
      prev_wp++;
    }
    int wp2 = (prev_wp + 1) % maps_x.size();
    int wp3 = (wp2 + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    double heading_2 = atan2((maps_y[wp3]-maps_y[wp2]),(maps_x[wp3]-maps_x[wp2]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);
    double seg_s_2 = (maps_s[wp2]-s);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double seg_x_2 = maps_x[wp2]-seg_s_2*cos(heading_2);
    double seg_y_2 = maps_y[wp2]-seg_s_2*sin(heading_2);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    cout << "[" << i << "] " << s <<  " = " << prev_wp << ":" << maps_s[prev_wp] << " - " << wp2 << ":" << maps_s[wp2] << endl;
    cout << "heading = " << heading << ", seg_s = " << seg_s << endl;
    cout << "seg_x = " << seg_x << ", seg_y = " << seg_y << endl;
    cout << "heading_2 = " << heading_2  << ", seg_s_2 = " << seg_s_2 << endl;
    cout << "seg_x_2 = " << seg_x_2 << ", seg_y_2 = " << seg_y_2 << endl;
  }



  plt::subplot(4, 1, 2);
  plt::title("Speed (v)");

  vector<double> SS_v;
  vector<double> DD_v;
  for (int i = 0; i < TT.size(); ++i) {
    SS_v.push_back(poly_calc(s_coeffs, TT[i], 1));
    DD_v.push_back(poly_calc(d_coeffs, TT[i], 1));
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
  double total_acc = 0.0;
  for (int i = 0; i < TT.size(); ++i) {
    double a = poly_calc(s_coeffs, TT[i], 2);
    SS_a.push_back(a);
    DD_a.push_back(poly_calc(d_coeffs, TT[i], 2));
    total_acc += abs(a * timestep);
  }
  plt::plot(TT, SS_a, "bo");
  cout << "total_acc = " << total_acc << endl;

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
  double total_jerk = 0.0;
  for (int i = 0; i < TT.size(); ++i) {
    double jerk = poly_calc(s_coeffs, TT[i], 3);
    SS_j.push_back(jerk);
    DD_j.push_back(poly_calc(d_coeffs, TT[i], 3));
    total_jerk += abs(jerk * timestep);
  }
  plt::plot(TT, SS_j, "bo");
  cout << "total_jerk = " << total_jerk << endl;

  // Annotate jerk
  for (int i = 0; i < TT.size(); i += int(1.0/timestep)) {
    ostringstream oss;
    oss << "  " << TT[i];
    plt::annotate(oss.str(), TT[i], SS_j[i]);
  }

  plt::show();



  // cout << "sensor fusion = " << sensor_fusion.dump(2) << endl;

  //cout << "pos = " << car_x << ", " << car_y << ", " << car_yaw << endl;
  cout << "s,d = " << car_s << ", " << car_d << endl;

  cout << endl <<  "<<<< END STATIC TEST!!!" << endl << endl;


}




#endif //PATH_PLANNING_TEST_LOCAL_H
