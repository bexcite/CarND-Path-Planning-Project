//
// Created by Pavlo Bashmakov on 8/3/17.
//

#ifndef PATH_PLANNING_TEST_LOCAL_H
#define PATH_PLANNING_TEST_LOCAL_H


#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <algorithm>
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
#include "constants.h"
#include "helpers_plot.h"


using namespace std;


// for convenience
using json = nlohmann::json;

namespace plt = matplotlibcpp;


void testLocalTrajectories(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {

  cout << endl << ">>> TESTING on STATIC DATA !!!!! [trajectories]" << endl << endl;

  json j = read_log();

  cout << "read from logFile " << j.size() << " lines" << endl;

  unsigned long start = 0; //154 (1046, 873)
  unsigned long end = start + 5; //j.size();

  bool plot_first = true;
  int plot_first_max = 1;


  int plot_first_cnt = 0;


  double t = 0;

  vector<double> XX;
  vector<double> YY;
  vector<double> TT;

  vector<double> SS;
  vector<double> DD;

  vector<double> end_SS;
  vector<double> end_DD;

  int N = min(end, j.size());


  for (int i = start; i < N; ++i) {

    cout << " ======= " << i << " ===========" << endl;

    double car_x = j[i]["car_x"];
    double car_y = j[i]["car_y"];
    double car_yaw = deg2rad(j[i]["car_yaw"]);
    double car_s = j[i]["car_s"];
    double car_d = j[i]["car_d"];
    vector<double> prev_x = j[i]["prev_x"];
    vector<double> prev_y = j[i]["prev_y"];
    double end_path_s = j[i]["end_path_s"];
    double end_path_d = j[i]["end_path_d"];
    double dt = j[i]["dt"];
    double car_speed = j[i]["car_speed"];
    Trajectory car_traj;
    car_traj.s_coeffs = json_read_vector(j[i]["s_coeffs"]);
    car_traj.d_coeffs = json_read_vector(j[i]["d_coeffs"]);
    car_traj.T = j[i]["traj_t"];

    if (i > 0) {
      XX.push_back(car_x);
      YY.push_back(car_y);
      SS.push_back(car_s);
      DD.push_back(car_d);
      end_SS.push_back(end_path_s);
      end_DD.push_back(end_path_d);
      t += dt;
      TT.push_back(t);
    }


    cout << "car x,y = " << car_x << ", " << car_y << endl;
    cout << "car s,d = " << car_s << ", " << car_d << endl;
    cout << "end_path s,d = " << end_path_s << ", " << end_path_d << endl;
    cout << "prev.size = " << prev_x.size() << endl;
    cout << "traj = " << car_traj.str() << endl;
    cout << "traj_end s,d = " << poly_calc(car_traj.s_coeffs, car_traj.T) << ", " << poly_calc(car_traj.d_coeffs, car_traj.T) << endl;

    auto acc_stats = traj_stats_acc(car_traj);
    auto j_stats = traj_stats_jerk(car_traj);
    cout << "acc_per_sec = " << acc_stats[0] << ", max_acc = " << acc_stats[1] << endl;
    cout << "jerk_per_sec = " << j_stats[0] << ", max_jerk = " << j_stats[1] << endl;


    // Plot detailed trajectory
    if (plot_first && plot_first_cnt < plot_first_max) {
      plot_traj(car_traj);
      ++plot_first_cnt;
      if (plot_first_cnt == plot_first_max) plot_first = false;
    }

    auto xy = getXYPathFromTraj(car_traj, maps_s, maps_x, maps_y);


    double car_l = car_speed * 0.05;
    plt::plot({car_x, car_x + car_l * cos(car_yaw)}, {car_y, car_y + car_l * sin(car_yaw)}, "r-");
    plt::plot({car_x}, {car_y}, "ro");

    // Annotate car
    plt::annotate(to_string(i), car_x, car_y);



    /*
    plt::ylim(car_y - 1, car_y + 4);
    plt::xlim(car_x - 1, car_x + 4);
    plt::grid(true);
     */

    // Traj in XY
    plt::plot(xy[0], xy[1], "bo");

    // Prev points
    plt::plot(prev_x, prev_y, "ro");

    int planHorizon = 50;
    int connectLength = 5;

    if (prev_x.size() > 0 && prev_x.size() <= planHorizon) {
      auto xy_conn = getXYPathConnected(connectLength, prev_x, prev_y, car_traj, maps_s, maps_x, maps_y);
      plt::plot(xy_conn[0], xy_conn[1], "g>");
    }




    /*

    // ============ Combine paths ==========================

    int forwardN = 5;


    auto traj_data = getSDbyTraj(car_traj, TRAJ_TIMESTEP); // s,d,t
//    cout << "traj_data.size = " << traj_data[0].size() << endl;
//    cout << "traj_data[0] = " << traj_data[2][0] << ", " << traj_data[2][traj_data[2].size()-1] << endl;

    auto xy_traj = getXY(traj_data[0], traj_data[1], maps_s, maps_x, maps_y);

    vector<double> xx_n;
    vector<double> yy_n;
    vector<double> tt_n;

    if (forwardN > 0) {
      // Add one first point
      tt_n.push_back(-PATH_TIMESTEP * forwardN);
      xx_n.push_back(prev_x[0]);
      yy_n.push_back(prev_y[0]);
    }

    // Add first forwardN steps of previous_path

//    for (int i = forwardN; i > 0; --i) {
//      tt_n.push_back(- PATH_TIMESTEP * i);
//      xx_n.push_back(previous_path_x[forwardN - i]);
//      yy_n.push_back(previous_path_y[forwardN - i]);
//    }


    // Add points from new traj
    for (int i = 0; i < traj_data[0].size(); ++i) {
      tt_n.push_back(traj_data[2][i]);
      xx_n.push_back(xy_traj[0][i]);
      yy_n.push_back(xy_traj[1][i]);
    }

    plt::plot(xx_n, yy_n, "y*");

//            next_x_vals = xy[0];
//            next_y_vals = xy[1];


    //double T = ss.size() * timestep;
    double T = car_traj.T;

//    cout << "xx_n, yy_n" << endl;

//    print_vals(xx_n, yy_n, 100);


//  vector<double> TT;
//  for (int i = 0; i < ss.size(); ++i) {
//    TT.push_back((double)i * timestep);
//  }

    // Makine it all to splines
    tk::spline splX;
    splX.set_points(tt_n, xx_n);

    tk::spline splY;
    splY.set_points(tt_n, yy_n);

    vector<double> XX_smooth;
    vector<double> YY_smooth;
    vector<double> TT;
    double t = - PATH_TIMESTEP * forwardN;
    double timestep2 = PATH_TIMESTEP; // path requirements
    while (t <= T) {
      XX_smooth.push_back(splX(t));
      YY_smooth.push_back(splY(t));
      TT.push_back(t);
      t += timestep2;
    }


    plt::ylim(YY_smooth[0]-1, YY_smooth[0]+5);
    plt::xlim(XX_smooth[0]-2, XX_smooth[0]+20);
    plt::grid(true);

    plt::plot(XX_smooth, YY_smooth, "g>");

    // =========== << Combine Paths ============
    */








  }

  plt::show();


  plt::subplot(4, 1, 1);
  plt::plot(TT, XX, "bo");
  plt::subplot(4, 1, 2);
  plt::plot(TT, YY, "bo");
//  plt::subplot(4, 1, 3);
//  plt::plot(TT, SS, "bo");
//  plt::subplot(4, 1, 4);
//  plt::plot(TT, DD, "bo");
  plt::subplot(4, 1, 3);
  plt::plot(TT, end_SS, "bo");
  plt::subplot(4, 1, 4);
  plt::plot(TT, end_DD, "bo");


  plt::show();



  cout << endl <<  "<<<< END STATIC TEST!!! [trajectories]" << endl << endl;



}


void testLocalStored(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {

  cout << endl << ">>> TESTING on STATIC DATA !!!!! [stored]" << endl << endl;

  json j = read_log();

  cout << "read from logFile " << j.size() << " lines" << endl;


  unsigned long start = 66; //154
  unsigned long end = start + 5; //j.size();

  bool plot_first = false;
  int plot_first_cnt = 0;
  int plot_first_max = 3;

  double t = 0;

  vector<double> XX;
  vector<double> YY;
  vector<double> TT;

  vector<double> SS;
  vector<double> DD;

  int N = min(end, j.size());

  for (int i = start; i < N; ++i) {


    double car_x = j[i]["car_x"];
    double car_y = j[i]["car_y"];
    double car_yaw = deg2rad(j[i]["car_yaw"]);
    double car_s = j[i]["car_s"];
    double car_d = j[i]["car_d"];
    vector<double> prev_x = j[i]["prev_x"];
    vector<double> prev_y = j[i]["prev_y"];
    double dt = j[i]["dt"];
    double car_speed = j[i]["car_speed"];
    Trajectory car_traj;
    car_traj.s_coeffs = json_read_vector(j[i]["s_coeffs"]);
    car_traj.d_coeffs = json_read_vector(j[i]["d_coeffs"]);
    car_traj.T = j[i]["traj_t"];

    if (i > 0) {
      XX.push_back(car_x);
      YY.push_back(car_y);
      SS.push_back(car_s);
      DD.push_back(car_d);
      t += dt;
      TT.push_back(t);
    }

    if (plot_first && plot_first_cnt < plot_first_max) {
      plot_traj(car_traj);
      ++plot_first_cnt;
      if (plot_first_cnt == plot_first_max) plot_first = false;
    }

    auto xy = getXYPathFromTraj(car_traj, maps_s, maps_x, maps_y);

//    cout << "car x,y = " << car_x << ", " << car_y << endl;
//    cout << "traj = " << car_traj.str() << endl;

    auto acc_stats = traj_stats_acc(car_traj);
    cout << "acc_per_sec = " << acc_stats[0] << ", max_acc = " << acc_stats[1] << endl;


    double car_l = car_speed * 0.05;
    plt::plot({car_x, car_x + car_l * cos(car_yaw)}, {car_y, car_y + car_l * sin(car_yaw)}, "r-");
    plt::plot({car_x}, {car_y}, "ro");

    // Traj in XY
    plt::plot(xy[0], xy[1], "bo");

    // Prev points
    plt::plot(prev_x, prev_y, "ro");


    // ============ Combine paths ==========================
    /*
    int forwardN = 0;


    auto traj_data = getSDbyTraj(car_traj, TRAJ_TIMESTEP); // s,d,t
//    cout << "traj_data.size = " << traj_data[0].size() << endl;
//    cout << "traj_data[0] = " << traj_data[2][0] << ", " << traj_data[2][traj_data[2].size()-1] << endl;

    auto xy_traj = getXY(traj_data[0], traj_data[1], maps_s, maps_x, maps_y);

    vector<double> xx_n;
    vector<double> yy_n;
    vector<double> tt_n;

    if (forwardN > 0) {
      // Add one first point
      tt_n.push_back(-PATH_TIMESTEP * forwardN);
      xx_n.push_back(prev_x[0]);
      yy_n.push_back(prev_y[0]);
    }

    // Add first forwardN steps of previous_path

//    for (int i = forwardN; i > 0; --i) {
//      tt_n.push_back(- PATH_TIMESTEP * i);
//      xx_n.push_back(previous_path_x[forwardN - i]);
//      yy_n.push_back(previous_path_y[forwardN - i]);
//    }


    // Add points from new traj
    for (int i = 0; i < traj_data[0].size(); ++i) {
      tt_n.push_back(traj_data[2][i]);
      xx_n.push_back(xy_traj[0][i]);
      yy_n.push_back(xy_traj[1][i]);
    }

    plt::plot(xx_n, yy_n, "yo");

//            next_x_vals = xy[0];
//            next_y_vals = xy[1];


    //double T = ss.size() * timestep;
    double T = car_traj.T;

//    cout << "xx_n, yy_n" << endl;

//    print_vals(xx_n, yy_n, 100);


//  vector<double> TT;
//  for (int i = 0; i < ss.size(); ++i) {
//    TT.push_back((double)i * timestep);
//  }

    // Makine it all to splines
    tk::spline splX;
    splX.set_points(tt_n, xx_n);

    tk::spline splY;
    splY.set_points(tt_n, yy_n);

    vector<double> XX_smooth;
    vector<double> YY_smooth;
    vector<double> TT;
    double t = - PATH_TIMESTEP * forwardN;
    double timestep2 = PATH_TIMESTEP; // path requirements
    while (t <= T) {
      XX_smooth.push_back(splX(t));
      YY_smooth.push_back(splY(t));
      TT.push_back(t);
      t += timestep2;
    }


    plt::ylim(YY_smooth[0]-1, YY_smooth[0]+5);
    plt::xlim(XX_smooth[0]-2, XX_smooth[0]+20);
    plt::grid(true);

    plt::plot(XX_smooth, YY_smooth, "g+");
    */

    // =================================




    plt::annotate(to_string(i), car_x, car_y);


    plt::show();


  }





  plt::subplot(4, 1, 1);
  plt::plot(TT, XX, "bo");
  plt::subplot(4, 1, 2);
  plt::plot(TT, YY, "bo");
  plt::subplot(4, 1, 3);
  plt::plot(TT, SS, "bo");
  plt::subplot(4, 1, 4);
  plt::plot(TT, DD, "bo");
  plt::show();



  cout << endl <<  "<<<< END STATIC TEST!!! [stored]" << endl << endl;

}



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

  // Curved part
//  vector<double> s_start = {145.0, car_speed, 0.0}; // {car_s, car_speed, 0};
//  vector<double> s_end = {155.0, 3.0, 0.0}; // {car_s+100, 15, 0};

  vector<double> s_start = {car_s, 13, 0.0}; // {car_s, car_speed, 0};
  vector<double> s_end = {car_s+100, 15.0, 0.0}; // {car_s+100, 15, 0};

  vector<double> d_start = {car_d, 0.0, 0.0};
  vector<double> d_end = {car_d, 0.0, 0.0};

  double T = 7.0;

  /*
  auto s_coeffs = JMT(s_start, s_end, T);
  auto d_coeffs = JMT(d_start, d_end, T);
  */
  auto traj = getJMT(s_start, s_end, d_start, d_end, T);

  cout << "Trajectory = " << traj.str() << endl;

  print_coeffs("s_coeffs : ", traj.s_coeffs);
  print_coeffs("d_coeffs : ", traj.d_coeffs);

  print_coeffs("s_coeffs d1 : ", differentiate(traj.s_coeffs));
  print_coeffs("d_coeffs d1 : ", differentiate(traj.d_coeffs));


  auto clk = chrono::high_resolution_clock::now();
  double dt;

  clk = chrono::high_resolution_clock::now();

  double timestep = TRAJ_TIMESTEP;
  double t = 0.0;
  vector<double> SS;
  vector<double> DD;
  vector<double> TT;
  while (t <= T+timestep) {
    double sx = poly_calc(traj.s_coeffs, t);
    double dx = poly_calc(traj.d_coeffs, t);
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

  auto xy_smooth = getXYPath(SS, DD, TT, maps_s, maps_x, maps_y);
  XX_smooth = xy_smooth[0];
  YY_smooth = xy_smooth[1];

  // Get XY to Frenet transform
  /*
  tk::spline splXY;
  splXY.set_points(XX, YY);

  vector<double> SS_b;
  vector<double> DD_b;
  vector<double> prev_sd;
  for (int i = 0; i < XX.size(); ++i) {
    double th = car_yaw;
    cout << "th = " << th << endl;
    auto sd = getFrenet(XX[i], YY[i], th, maps_x, maps_y);
    if (i > 0) {
      double Dxy = distance(XX[i-1], YY[i-1], XX[i], YY[i]);
      double Dsd = distance(prev_sd[0], prev_sd[1], sd[0], sd[1]);
      cout << "ratio fre: " << Dxy/Dsd << endl;
      double ds = (sd[0] - prev_sd[0]) * (Dxy/Dsd);
      double dd = (sd[1] - prev_sd[1]) * (Dxy/Dsd);
      sd[0] = prev_sd[0] + ds;
      sd[1] = prev_sd[1] + dd;

    }
    SS_b.push_back(sd[0]);
    DD_b.push_back(sd[1]);
    prev_sd = sd;
  }
   */

  dt = chrono::duration<double>(chrono::high_resolution_clock::now() - clk).count();
  cout << "DT smoothing = " << dt << endl;

  plt::subplot(2, 1, 1);
  plt::title("XY and XY_smooth");
  plt::plot(XX, YY, "bo");
  plt::plot(XX1, YY1, "rx");
//  plt::plot(XX2, YY2, "bo");

  plt::subplot(2, 1, 2);
  plt::plot(XX_smooth, YY_smooth, "ro");
//  plt::plot(SS_b, DD_b, "bo");
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
    oss << " " << TT[i];
    plt::annotate(oss.str(), SS[i], DD[i] + 0.2);
  }

  /*
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
  */


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
  double total_acc = 0.0;
  for (int i = 0; i < TT.size(); ++i) {
    double a = poly_calc(traj.s_coeffs, TT[i], 2);
    SS_a.push_back(a);
    DD_a.push_back(poly_calc(traj.d_coeffs, TT[i], 2));
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
    double jerk = poly_calc(traj.s_coeffs, TT[i], 3);
    SS_j.push_back(jerk);
    DD_j.push_back(poly_calc(traj.d_coeffs, TT[i], 3));
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
