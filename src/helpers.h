//
// Created by Pavlo Bashmakov on 8/3/17.
//

#ifndef PATH_PLANNING_HELPERS_H_H
#define PATH_PLANNING_HELPERS_H_H


#include <fstream>
#include <math.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <fstream>
//#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "json.hpp"
//#include "matplotlibcpp.h"
//#include <typeinfo>
#include <unistd.h>
#include <cassert>
#include "spline.h"
#include "constants.h"

using namespace std;

// for convenience
using json = nlohmann::json;



/* ==== Some Forward Definitions =========== */

string str_coeffs(vector<double> coeffs);
double poly_calc(vector<double> coeffs, double x, int order);


/* === Trajectory description ==== */
struct Trajectory {
  vector<double> s_coeffs;
  vector<double> d_coeffs;
  double T;
  string str() {
    ostringstream oss;
    oss << "(" << str_coeffs(s_coeffs) << ", " << str_coeffs(d_coeffs) << ", " << T << ")";
    return oss.str();
  }
};



/* ==== HandlerState ==== */
class HandlerState {
public:

  HandlerState() {
    prev_clk = chrono::high_resolution_clock::now();
    start_clk = chrono::high_resolution_clock::now();
    initialized = false;
  }

  virtual ~HandlerState() {}

  chrono::time_point<chrono::high_resolution_clock> prev_clk;
  chrono::time_point<chrono::high_resolution_clock> start_clk;

  bool initialized = false;

  bool ready = false;

  bool path = false;

  double dt;
  double t;

  double prev_v;

  double acc;

  unsigned int cnt = 0;

  Trajectory prev_traj;

  vector<vector<double> > prev_xy;
  int prev_next_idx = 0;

  vector<double> vec_car_x;
  vector<double> vec_car_y;
  vector<double> vec_car_yaw;
  vector<double> vec_car_s;
  vector<double> vec_car_d;
  vector<double> vec_t;

  void tick() {

    // Calc dt time
    auto clk = chrono::high_resolution_clock::now();
    dt = chrono::duration<double>(clk - prev_clk).count();
    t = chrono::duration<double>(clk - start_clk).count();
    prev_clk = clk;



    if (initialized && !ready) {
      ready = true;
    }
    if (!initialized) {
      initialized = true;
      start_clk = clk;
    }

    cnt++;
  }

  void save(double car_x, double car_y, double car_yaw, double car_s, double car_d) {
    if (ready) {
      vec_car_x.push_back(car_x);
      vec_car_y.push_back(car_y);
      vec_car_yaw.push_back(car_yaw);
      vec_car_s.push_back(car_s);
      vec_car_d.push_back(car_d);
      vec_t.push_back(t);
    }

  }

};






// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}



double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}


int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}


vector<vector<double> > getXY(vector<double> ss, vector<double> dd, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, bool adjust = true) {

  vector<double> xx;
  vector<double> yy;
  vector<double> prev_xy;
  for (int i = 0; i < ss.size(); ++i) {
    vector<double> xy = getXY(ss[i], dd[i], maps_s, maps_x, maps_y);
    if (adjust && i > 0) {
      // adjust next point to be on the same dist as s,d points
      double Dsd = distance(ss[i-1], dd[i-1], ss[i], dd[i]);
      double Dxy = distance(prev_xy[0], prev_xy[1], xy[0], xy[1]);
//      cout << "ratio: " << Dsd/Dxy << endl;
      double dx = (xy[0] - prev_xy[0]) * (Dsd/Dxy);
      double dy = (xy[1] - prev_xy[1]) * (Dsd/Dxy);
      xy[0] = prev_xy[0] + dx;
      xy[1] = prev_xy[1] + dy;
    }
    xx.push_back(xy[0]);
    yy.push_back(xy[1]);
    prev_xy = xy;
  }

  return {xx, yy};
}


vector<vector<double> > getXYPath(vector<double> ss, vector<double> dd, vector<double> tt, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {

  auto xy = getXY(ss, dd, maps_s, maps_x, maps_y);

//            next_x_vals = xy[0];
//            next_y_vals = xy[1];


  //double T = ss.size() * timestep;
  double T = tt[tt.size() - 1];


//  vector<double> TT;
//  for (int i = 0; i < ss.size(); ++i) {
//    TT.push_back((double)i * timestep);
//  }

  // Spline Smoothing XY line
  tk::spline splX;
  splX.set_points(tt, xy[0]);

  tk::spline splY;
  splY.set_points(tt, xy[1]);

  vector<double> XX_smooth;
  vector<double> YY_smooth;
  vector<double> TT;
  double t = tt[0];
  double timestep2 = PATH_TIMESTEP; // path requirements
  while (t <= T) {
    XX_smooth.push_back(splX(t));
    YY_smooth.push_back(splY(t));
    TT.push_back(t);
    t += timestep2;
  }

//  cout << "smooth_TT[0] = " << TT[0] << ", smooth_TT[end] = " << TT[TT.size()-1] << endl;

  return {XX_smooth, YY_smooth, TT};

}




// Differentiate
vector<double> differentiate(vector<double> coeffs) {
  vector<double> dcoeffs;
  for (int i = 1; i < coeffs.size(); ++i) {
    dcoeffs.push_back(i * coeffs[i]);
  }
  return dcoeffs;
}



// Calculate polynomial value given 'coeffs' and point 'x'
double poly_calc(vector<double> coeffs, double x, int order = 0) {
  vector<double> dcoeffs = coeffs;
  for (int i = 0; i < order; ++i) {
    dcoeffs = differentiate(dcoeffs);
  }
  double total = 0.0;
  for (int i = 0; i < dcoeffs.size(); ++i) {
    total += dcoeffs[i] * pow(x, i);
  }
  return total;
}



vector<vector<double> > getSDbyTraj(Trajectory traj, double timestep = TRAJ_TIMESTEP, double timeShift = 0.0) {

  vector<double> SS;
  vector<double> DD;
  vector<double> TT;

  double t = timeShift;
//  double timestep = TRAJ_TIMESTEP;
  while (t < traj.T /* + timestep*/) {
    double sx = poly_calc(traj.s_coeffs, t);
    double dx = poly_calc(traj.d_coeffs, t);
    SS.push_back(sx);
    DD.push_back(dx);
    TT.push_back(t);
    t += timestep;
  }

  // Add last point
//  SS.push_back(poly_calc(traj.s_coeffs, traj.T));
//  DD.push_back(poly_calc(traj.d_coeffs, traj.T));
//  TT.push_back(traj.T);

  return {SS, DD, TT};
}

// Jerk Minimizing Trajectory
// Returns coefficients for quintic polynomial
vector<double> JMT(vector<double> start, vector<double> end, double T) {
  double T2 = T*T;
  double T3 = T2*T;
  double T4 = T3*T;
  double T5 = T4*T;

  Eigen::MatrixXd A(3, 3);
  A << T3,   T4,    T5,
          3*T2, 4*T3,  5*T4,
          6*T,  12*T2, 20*T3;

  Eigen::VectorXd B(3,1);
  B << end[0] - (start[0] + start[1]*T + 0.5 * start[2] * T2),
          end[1] - (start[1] + start[2]*T),
          end[2] - start[2];

  Eigen::VectorXd R = A.inverse() * B;

  return {start[0], start[1], start[2]/2, R(0), R(1), R(2)};
}


Trajectory getJMT(vector<double> s_start, vector<double> s_end, vector<double> d_start, vector<double> d_end, double T) {
//  double T = 7.0;

  Trajectory traj;
  traj.s_coeffs = JMT(s_start, s_end, T);
  traj.d_coeffs = JMT(d_start, d_end, T);
  traj.T = T;
  return traj;
}







void print_coeffs(std::string s, vector<double> coeffs) {
  cout << s;
  if (coeffs.empty()) {
    cout << " - empty" << endl;
    return;
  }
  for (int i = 0; i < coeffs.size()-1; ++i) {
    cout << coeffs[i] << ", ";
  }
  cout << coeffs[coeffs.size()-1] << endl;
}


// Conver coeffs to string reprepsentation
string str_coeffs(vector<double> coeffs) {
  ostringstream oss;
  if (coeffs.empty()) return "[]";
  oss << "[";
  for (int i = 0; i < coeffs.size() - 1; ++i) {
    oss << coeffs[i] << ", ";
  }
  oss << coeffs[coeffs.size() - 1] << "]";
  return oss.str();

}


void print_vals(vector<double> X, vector<double> Y, int max_num = 10) {
  int m = min(max_num, (int)X.size());
  for (int i = 0; i < m; ++i) {
    cout << "[" << i << "] = " << X[i] << ", " << Y[i] << endl;
  }
  if (m < X.size()) {
    cout << ". . . ." << endl;
  }
}


vector<double> getTT(double timestep, int num) {
  vector<double> TT;
  for (int i = 0; i < num; ++i) {
    TT.push_back(i * timestep);
  }
  return TT;
}


double findTimeInTrajByXY(Trajectory traj, double car_x, double car_y, double car_yaw, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {

  auto traj_data = getSDbyTraj(traj, TRAJ_TIMESTEP);
  auto xy = getXYPath(traj_data[0], traj_data[1], traj_data[2], maps_s, maps_x, maps_y);

//  cout << "findTime: xy = " << endl;
//  print_vals(xy[0], xy[1]);


//  int min_i = ClosestWaypoint(car_x, car_y, xy[0], xy[1]);
  int min_i = NextWaypoint(car_x, car_y, car_yaw, xy[0], xy[1]);
  if (min_i == 0) return 0.0;

  double t = (min_i - 1) * PATH_TIMESTEP;

  /* not needed due to how simulator works :(
   * Spend too much time on these timing issues ...
  double Dxy = distance(xy[0][min_i-1], xy[1][min_i-1], xy[0][min_i], xy[1][min_i]);
  double d = distance(xy[0][min_i-1], xy[1][min_i-1], car_x, car_y);

  t += d/Dxy * PATH_TIMESTEP;
   */

  return t;

}


vector<vector<double> > getXYPathFromTraj(Trajectory traj, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, double timeShift = 0.0) {

  auto traj_data = getSDbyTraj(traj, TRAJ_TIMESTEP * 2 /*, timeShift*/); // s,d,t
//  cout << "traj_data.size = " << traj_data[0].size() << endl;
//  cout << "traj_data[0] = " << traj_data[2][0] << ", " << traj_data[2][traj_data[2].size()-1] << endl;

  auto xy = getXYPath(traj_data[0], traj_data[1], traj_data[2], maps_s, maps_x, maps_y);

  double t = 0.0;
  while (t < timeShift) {
    xy[0].erase(xy[0].begin());
    xy[1].erase(xy[1].begin());
    t += PATH_TIMESTEP;
  }

  return xy;
}


vector<vector<double> > getXYPathConnected(vector<double> prev_x, vector<double> prev_y, Trajectory traj, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, double timeShift = 0.0) {

  /*
  auto traj_data = getSDbyTraj(traj, TRAJ_TIMESTEP * 2); // s,d,t
//  cout << "traj_data.size = " << traj_data[0].size() << endl;
//  cout << "traj_data[0] = " << traj_data[2][0] << ", " << traj_data[2][traj_data[2].size()-1] << endl;

  auto xy_traj = getXY(traj_data[0], traj_data[1], maps_s, maps_x, maps_y);
  */

  auto xy_traj = getXYPathFromTraj(traj, maps_s, maps_x, maps_y, timeShift);

  if (prev_x.size() > 0) {
    int last_idx = prev_x.size() - 1;
    double disp_x = prev_x[last_idx] - xy_traj[0][0];
    double disp_y = prev_y[last_idx] - xy_traj[1][0];
    // Move next path so it's connected with prev path
    for (int i = 0; i < xy_traj[0].size(); ++i) {
      xy_traj[0][i] += disp_x;
      xy_traj[1][i] += disp_y;
    }
  }

  return {xy_traj[0], xy_traj[1]};


//  vector<double> xx_n;
//  vector<double> yy_n;
//  vector<double> tt_n;
//
//  if (conn_len > 0 && prev_x.size() > 0) {
//    int N = min(conn_len, (int)prev_x.size());
//    int idx = prev_x.size() - N;
//    // Add one first point
//    tt_n.push_back(-PATH_TIMESTEP * N);
//    xx_n.push_back(prev_x[idx]);
//    yy_n.push_back(prev_y[idx]);
//  }

  // Add last conn_len steps of previous_path

//    for (int i = 0; i < conn_len; ++i) {
//      tt_n.push_back(- PATH_TIMESTEP * (conn_len - i));
//      int idx = prev_x.size() - conn_len + i;
//      xx_n.push_back(prev_x[idx]);
//      yy_n.push_back(prev_y[idx]);
//    }

/*
  // Add points from new traj
  for (int i = 0; i < traj_data[0].size(); ++i) {
    tt_n.push_back(traj_data[2][i]);
    xx_n.push_back(xy_traj[0][i]);
    yy_n.push_back(xy_traj[1][i]);
  }

  double T = traj.T;

  // Makine it all to splines
  tk::spline splX;
  splX.set_points(tt_n, xx_n);

  tk::spline splY;
  splY.set_points(tt_n, yy_n);

  // Calc displacement of first point from last prev point
  double disp_x =

  vector<double> XX_smooth;
  vector<double> YY_smooth;
  vector<double> TT;
  double t = timeShift;
  double timestep2 = PATH_TIMESTEP; // path requirements
  while (t <= T) {
    XX_smooth.push_back(splX(t));
    YY_smooth.push_back(splY(t));
    TT.push_back(t);
    t += timestep2;
  }
  */


//  return {XX_smooth, YY_smooth, TT};
}




void prepare_log() {
  ofstream logFile ("log.out", ios::out);
  if (logFile.is_open()) {
    //logFile << "--- start ----" << endl;
    logFile.close();
  }
}



void add_to_log(double dt, double dt_d,  double car_x, double car_y, double car_yaw, double car_s, double car_d, double car_speed, Trajectory traj, vector<double> prev_traj_x, vector<double> prev_traj_y, int prev_next_idx, json prev_x, json prev_y, double end_path_s, double end_path_d, vector<double> next_x_vals, vector<double> next_y_vals, json sensor_fusion) {
  ofstream logFile ("log.out", ios::out | ios::app);
  if (logFile.is_open()) {
    json j;
    j["dt"] = dt;
    j["dt_d"] = dt_d;
    j["car_x"] = car_x;
    j["car_y"] = car_y;
    j["car_yaw"] = car_yaw;
    j["car_s"] = car_s;
    j["car_d"] = car_d;
    j["end_path_s"] = end_path_s;
    j["end_path_d"] = end_path_d;
    j["car_speed"] = car_speed; // m/s
    j["s_coeffs"] = traj.s_coeffs;
    j["d_coeffs"] = traj.d_coeffs;
    j["traj_t"] = traj.T;
    j["prev_x"] = prev_x;
    j["prev_y"] = prev_y;
    j["prev_traj_x"] = prev_traj_x;
    j["prev_traj_y"] = prev_traj_y;
    j["prev_next_idx"] = prev_next_idx;
    j["next_x_vals"] = next_x_vals;
    j["next_y_vals"] = next_y_vals;
    j["sensor_fusion"] = sensor_fusion;
    logFile << j << endl;
    logFile.close();
  } else {
    cout << "ERROR opening file!!!" << endl;
  }

}


vector<json> read_log() {
  cout << "read log" << endl;
  string line;
  ifstream logFile("log.out", ifstream::in);
  vector<json> j;
  cout << "read log 1" << endl;
  if (logFile.is_open()) {

    while (getline(logFile, line)) {
      auto jline = json::parse(line);
      j.push_back(jline);
    }

    logFile.close();

  } else {
    cout << "Unable to open logFile" << endl;
  }
  return j;
}


vector<double> json_read_vector(json j) {
  vector<double> r;
  for (int i = 0; i < j.size(); ++i) {
    r.push_back(j[i]);
  }
  return r;
}


vector<double> traj_stats_acc(Trajectory traj) {
  auto traj_data = getSDbyTraj(traj, PATH_TIMESTEP);
  vector<double> SS = traj_data[0];
  vector<double> DD = traj_data[1];
  vector<double> TT = traj_data[2];

  vector<double> s_coeffs_v = differentiate(traj.s_coeffs);
  vector<double> s_coeffs_a = differentiate(s_coeffs_v);

  double max_acc = 0;
  double total_acc = 0;
  for (int i = 0; i < TT.size(); ++i) {
    double a = poly_calc(s_coeffs_a, TT[i]);
    if (abs(a) > max_acc) {
      max_acc = abs(a);
    }
    total_acc += abs(a * PATH_TIMESTEP);
  }
  double acc_per_second = total_acc / traj.T;

  return {acc_per_second, max_acc};

}

vector<double> traj_stats_jerk(Trajectory traj) {
  auto traj_data = getSDbyTraj(traj, PATH_TIMESTEP);
  vector<double> SS = traj_data[0];
  vector<double> DD = traj_data[1];
  vector<double> TT = traj_data[2];

  vector<double> s_coeffs_v = differentiate(traj.s_coeffs);
  vector<double> s_coeffs_a = differentiate(s_coeffs_v);
  vector<double> s_coeffs_j = differentiate(s_coeffs_a);

  double max_j = 0;
  double total_j = 0;
  for (int i = 0; i < TT.size(); ++i) {
    double j = poly_calc(s_coeffs_j, TT[i]);
    if (abs(j) > max_j) {
      max_j = abs(j);
    }
    total_j += abs(j * PATH_TIMESTEP);
  }
  double j_per_second = total_j / traj.T;

  return {j_per_second, max_j};

}


Trajectory genTraj(double tLane, double tSpeed, vector<double> s_start, vector<double> d_start, double T = 5.0) {

  double end_d = (2.0 + 4.0 * tLane);

  double avg_v = distance(0.0, 0.0, s_start[1], d_start[1]);
  avg_v = avg_v + distance(0.0, 0.0, tSpeed, 0.0);
  avg_v = avg_v/2;

  double travel_dist = avg_v * T;


  double delta_d = end_d - d_start[0];
  double s_dist = sqrt(travel_dist * travel_dist - delta_d * delta_d);


  vector<double> s_end = {s_start[0] + s_dist, tSpeed, 0.0}; // {car_s+100, 15, 0};
  vector<double> d_end = {end_d, 0.0, 0.0};



  cout << "s_dist = " << s_dist << endl;
  cout << "avg_v = " << avg_v << endl;

  print_coeffs("s_start = ", s_start);
  print_coeffs("d_start = ", d_start);
  print_coeffs("s_end = ", s_end);
  print_coeffs("d_end = ", d_end);

  cout << "T = " << T << endl;

  auto traj = getJMT(s_start, s_end, d_start, d_end, T);

  return traj;


}



class SensorFusion {
public:
  int track_n = 10;
  vector<vector<vector<double> > > sensorData;

  vector<double> maps_s;
  vector<double> maps_x;
  vector<double> maps_y;

  SensorFusion(vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) :
          maps_s(maps_s), maps_x(maps_x), maps_y(maps_y) {
  }

  void add(vector<vector<double> > sensor_fusion, double dt_d) {
    if (sensorData.size() < sensor_fusion.size()) {
      sensorData.resize(sensor_fusion.size());
    }



    for (int i = 0; i < sensor_fusion.size(); ++i) {
      double d = sensor_fusion[i][6];

      if (d < 0.0 || 12.0 < d ) {
//        cout << "clear for i = " << i << endl;
        sensorData[i].clear();
        continue;
      }

      // Remove all but track_n - 1 records
//      cout << "erase elems ..." << endl;
//      cout << "erasse sensorData[i].size() = " << sensorData[i].size() << endl;
      while (sensorData[i].size() >= track_n ) {
        sensorData[i].erase(sensorData[i].begin());
      }

      vector<double> sfd = sensor_fusion[i];

      double x = sfd[1];
      double y = sfd[2];
      double vx = sfd[3];
      double vy = sfd[4];
      double s = sfd[5];
//      double d = sfd[6];

      double tt = 0.04;
      double x1 = x + tt * vx;
      double y1 = y + tt * vy;

      auto sd1 = getFrenet(x1, y1, atan2(vy, vx), maps_x, maps_y);
      double vs = (sd1[0] - s) / tt;
      double vd = (sd1[1] - d) / tt;

      sfd.push_back(vs);
      sfd.push_back(vd);

      // Add time
      sfd.push_back(dt_d);
      sensorData[i].push_back(sfd);
    }

    cout << "sf: cars = " << sensorData.size() << endl;
    cout << "sf: steps = " << sensorData[0].size() << endl;

  }

//  vector<vector<double> > getCarHistory(int idx) {
//    vector<vector<double> > history;
//    if (sensorData.size() < idx) return history;
//    return sensorData[idx];
//  }

  vector<vector<double> > getCarHistory(int idx) {
    vector<vector<double> > history(7);
    if (sensorData.size() < idx) return history;
    double t = 0.0;
    for (int i = 0; i < sensorData[idx].size(); ++i) {
      history[0].push_back(sensorData[idx][i][1]); // x
      history[1].push_back(sensorData[idx][i][2]); // y
      history[2].push_back(sensorData[idx][i][5]); // s
      history[3].push_back(sensorData[idx][i][6]); // d
      history[4].push_back(sensorData[idx][i][7]); // vs
      history[5].push_back(sensorData[idx][i][8]); // vd
      if (i > 0) {
        t += sensorData[idx][i][9]; // dt_d
      }
    }

    t = -t;
    for (int i = 0; i < sensorData[idx].size(); ++i) {
      if (i > 0) {
        t += sensorData[idx][i][9]; // dt_d
      }
//      cout << "push t = " << t << endl;
      history[6].push_back(t); // t
    }

//    cout << "t = " << t << endl;

    assert(t < abs(0.001));

    return history;
  }

  vector<vector<vector<double> > > getAllCarHistory() {
    vector<vector<vector<double> > > allHistory;
    for (int i = 0; i < sensorData.size(); ++i) {
      allHistory.push_back(getCarHistory(i));
    }
    return allHistory;
  }

  vector<double> getCarXY(int idx) {
    return {0.0, 0.0};
  }

  bool isCarActive(int idx) {
    if (sensorData.size() < idx) return false;
    if (sensorData[idx].size() > 0) return true;
    return false;
  }

  void print_car(int idx) {
    auto car_data = sensorData[idx];
    cout << "sf: car info [" << idx << "] : x, y, vx, vy, s, d" << endl;


    for(int i = 0; i < car_data.size(); ++i) {
      double x = car_data[i][1];
      double y = car_data[i][2];
      double vx = car_data[i][3];
      double vy = car_data[i][4];
      double s = car_data[i][5];
      double d = car_data[i][6];
      double dt_d = car_data[i][9];
      cout << x << ", " << y << ", " << vx << ", " << vy << ", " << s << ", " << d << ", " << dt_d << endl;
    }
  }

  vector<vector<double> > getTrajSD(int idx, double T = 5.0) {
    auto car_data = getCarHistory(idx);

    double s = car_data[2][car_data[2].size() - 1];
    double d = car_data[3][car_data[3].size() - 1];

    double vs = car_data[4][car_data[4].size() - 1];
    double vd = car_data[5][car_data[5].size() - 1];

    vector<double> SS;
    vector<double> DD;
    vector<double> TT;

    if (car_data[0].size() < 1) {
      return {SS, DD, TT};
    }

    double t = 0.0;
    while ( t <= T) {
      SS.push_back(s + t * vs);
      DD.push_back(d + t * vd);
      TT.push_back(t);
      t += PATH_TIMESTEP;
    }

    /*
    tk::spline splS;
    splS.set_points(car_data[2], car_data[0]); // t, s

    tk::spline splD;
    splD.set_points(car_data[2], car_data[1]); // d, s

    double t = 0.0;

    while ( t <= T) {
      SS.push_back(splS(t));
      DD.push_back(splD(t));
      TT.push_back(t);
      t += PATH_TIMESTEP;
    }
     */

    return {SS, DD, TT};

  }

  vector<vector<double> > getTrajXY(int idx, double T = 5.0) {

    auto sd_traj = getTrajSD(idx, T);

//    cout << "sd_traj.size = " << sd_traj[0].size() << endl;

    auto xy_traj = getXY(sd_traj[0], sd_traj[1], maps_s, maps_x, maps_y);

//    cout << "xy_traj.size = " << xy_traj[0].size() << endl;
//    print_coeffs("xy[0] = ", xy_traj[0]);
//    print_coeffs("xy[1] = ", xy_traj[1]);

    return {xy_traj[0], xy_traj[1], sd_traj[2]};
  }

  int size() {return sensorData.size();}


};




#endif //PATH_PLANNING_HELPERS_H_H
