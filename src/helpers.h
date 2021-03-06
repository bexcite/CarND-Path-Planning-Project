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
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "json.hpp"
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
    xx.push_back(xy[0]);
    yy.push_back(xy[1]);
    prev_xy = xy;
  }


  unsigned long n = 15;
  double prev_x = xx[0];
  double prev_y = yy[0];
  for (int i = 1; i < xx.size(); ++i) {
    int next_idx = min(i + n, xx.size() - 1);
    double Dsd = distance(ss[i - 1], dd[i - 1], ss[i], dd[i]);
    double Dxy = distance(prev_x, prev_y, xx[next_idx], yy[next_idx]);
    //      cout << "ratio: " << Dsd/Dxy << endl;
    double dx = (xx[next_idx] - prev_x) * (Dsd / Dxy);
    double dy = (yy[next_idx] - prev_y) * (Dsd / Dxy);
    xx[i] = prev_x + dx;
    yy[i] = prev_y + dy;
    prev_x = xx[i];
    prev_y = yy[i];
  }



  return {xx, yy};
}


vector<vector<double> > getXYPath(vector<double> ss, vector<double> dd, vector<double> tt, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {

  auto xy = getXY(ss, dd, maps_s, maps_x, maps_y); // TDOO: test remove

  // Laplassian smooth algorithms
//  int rounds = 1;
//  while (rounds > 0) {
//    for (int j = 1; j < xy[0].size() - 1; ++j) {
//      xy[0][j] = 0.5 * (xy[0][j - 1] + xy[0][j + 1]);
//      xy[1][j] = 0.5 * (xy[1][j - 1] + xy[1][j + 1]);
//    }
//    --rounds;
//  }


  double T = tt[tt.size() - 1];


  vector<double> fitX;
  vector<double> fitY;
  vector<double> fitT;

  unsigned long i = 0;
  unsigned long N = xy[0].size();
  while (i < N) {
    fitX.push_back(xy[0][i]);
    fitY.push_back(xy[1][i]);
    fitT.push_back(i*PATH_TIMESTEP);
    if (i < N - 1) {
      i = min(i + 15, N - 1);
    } else {
      break; // or ++i :), Looks weird ....
    }
  }


  // Spline Smoothing XY line
  tk::spline splX;
  splX.set_points(fitT, fitX); // tt, xy[0]

  tk::spline splY;
  splY.set_points(fitT, fitY); // xy[1]

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


  return {XX_smooth, YY_smooth, TT, xy[0], xy[1]};


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
  while (t < traj.T + 0.001 /* + timestep*/) {
    double sx = poly_calc(traj.s_coeffs, t);
    double dx = poly_calc(traj.d_coeffs, t);
    SS.push_back(sx);
    DD.push_back(dx);
    TT.push_back(t);
    t += timestep;
  }

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


// Help method for debug outpus
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

// Used for debug outputs
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

  auto traj_data = getSDbyTraj(traj, PATH_TIMESTEP);
  auto xy = getXYPath(traj_data[0], traj_data[1], traj_data[2], maps_s, maps_x, maps_y);


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

  auto traj_data = getSDbyTraj(traj, PATH_TIMESTEP); // s,d,t

  auto xy = getXYPath(traj_data[0], traj_data[1], traj_data[2], maps_s, maps_x, maps_y);

  double t = 0.0;
  while (t < timeShift) {
    xy[0].erase(xy[0].begin());
    xy[1].erase(xy[1].begin());
    t += PATH_TIMESTEP;
  }

  return xy;
}


// THIS IS AN OLD METHOD, look for getXYPathConnected1() :)
vector<vector<double> > getXYPathConnected(vector<double> prev_x, vector<double> prev_y, Trajectory traj, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, double timeShift = 0.0) {

  auto xy_traj = getXYPathFromTraj(traj, maps_s, maps_x, maps_y, timeShift);

  // Connect to previous path
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

  return xy_traj;

}



vector<vector<double> > getXYPathConnected1(vector<double> prev_x, vector<double> prev_y, double car_x, double car_y, double car_yaw, Trajectory traj, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y, double timeShift = 0.0) {


  auto traj_data = getSDbyTraj(traj, PATH_TIMESTEP); // s,d,t

  vector<double> ss = traj_data[0];
  vector<double> dd = traj_data[1];
  vector<double> tt = traj_data[2];

  vector<double> fitX;
  vector<double> fitY;
  vector<double> fitT;

  unsigned long prev_size = prev_x.size();
  unsigned long traj_size = ss.size();

  double start_x;
  double end_x;


  // Add first point from prev path
  if (prev_size > 0) {
    start_x = prev_x[0];
    fitX.push_back(prev_x[0]);
    fitY.push_back(prev_y[0]);
    double temp_t = -1 * (double)(prev_size*PATH_TIMESTEP);
    fitT.push_back(temp_t);
  }

  // Add middle point from prev path
  if (prev_size > 2) {
    fitX.push_back(prev_x[(int)prev_size/2]);
    fitY.push_back(prev_y[(int)prev_size/2]);
    double temp_t = -1 * (double)((prev_size-(int)prev_size/2)*PATH_TIMESTEP);
    fitT.push_back(temp_t);
  }

  // Add a point before the last in prev path
  if (prev_size > 3) {
    fitX.push_back(prev_x[prev_size-2]);
    fitY.push_back(prev_y[prev_size-2]);
    fitT.push_back(-1 * PATH_TIMESTEP);
  }

  // Add last point from prev path
  if (prev_size > 1) {
    start_x = prev_x[prev_size-1];
    fitX.push_back(prev_x[prev_size-1]);
    fitY.push_back(prev_y[prev_size-1]);
    fitT.push_back(0.0);
  }


  // Add first point as a car location if we don't have a prev path
  vector<double> xy;
  if (prev_size == 0) {
    xy = getXY(ss[0], dd[0], maps_s, maps_x, maps_y);
    start_x = xy[0];
    fitX.push_back(car_x); // xy[0]
    fitY.push_back(car_y); // xy[1]
    fitT.push_back(0.0);
  }


  // Add central point of trajectory
  xy = getXY(ss[(int)traj_size/2], dd[(int)traj_size/2], maps_s, maps_x, maps_y);
  fitX.push_back(xy[0]);
  fitY.push_back(xy[1]);
  fitT.push_back((double)((int)traj_size/2 * PATH_TIMESTEP));


  // Add last point of trajectory
  xy = getXY(ss[traj_size-1], dd[traj_size-1], maps_s, maps_x, maps_y);
  fitX.push_back(xy[0]);
  fitY.push_back(xy[1]);
  fitT.push_back((double)(traj_size-1) * PATH_TIMESTEP);


  end_x = xy[0];

  double delta_x = (end_x - start_x)/(traj_size + 1);


  // Makine it all to splines
  tk::spline splX;
  splX.set_points(fitT, fitX);
  tk::spline splY;
  splY.set_points(fitT, fitY);

  vector<double> pathX;
  vector<double> pathY;

  double t = 0.0;
  double delta_t = PATH_TIMESTEP;
  while (t <= tt[traj_size-1] + 0.001) {
    pathX.push_back(splX(t));
    pathY.push_back(splY(t));
    t += delta_t;
  }

  assert(tt.size() == pathX.size());

  // Check distance difference between sd-path and xt-path
  double sd_sum = 0;
  double xy_sum = 0;
  for (int i = 0; i < tt.size()-1; ++i) {
    double sd_dist = distance(ss[i], dd[i], ss[i+1], dd[i+1]);
    double xy_dist = distance(pathX[i], pathY[i], pathX[i+1], pathY[i+1]);
    sd_sum += sd_dist;
    xy_sum += xy_dist;
  }

  cout << "sd, xy sums = " << sd_sum << ", " << xy_sum << ", ratio = " << (sd_sum/xy_sum) << endl;


  if (xy_sum > sd_sum) {
    // resampling with lower/higher rates
    cout << "resampling ..." << endl;
    pathX.clear();
    pathY.clear();
    t = 0.0;
    delta_t = PATH_TIMESTEP * (sd_sum/xy_sum);
    while (t <= tt[traj_size-1] + 0.001) {
      pathX.push_back(splX(t));
      pathY.push_back(splY(t));
      t += delta_t;
    }
  }

  // Check distance difference [2]
  sd_sum = 0;
  xy_sum = 0;
  for (int i = 0; i < tt.size()-1; ++i) {
    double sd_dist = distance(ss[i], dd[i], ss[i+1], dd[i+1]);
    double xy_dist = distance(pathX[i], pathY[i], pathX[i+1], pathY[i+1]);
    sd_sum += sd_dist;
    xy_sum += xy_dist;
  }

  cout << "sd, xy sums = " << sd_sum << ", " << xy_sum << ", ratio = " << (sd_sum/xy_sum) << endl;

  return {pathX, pathY};

}



void prepare_log() {
  ofstream logFile ("log.out", ios::out);
  if (logFile.is_open()) {
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
        sensorData[i].clear();
        continue;
      }

      // Remove all but track_n - 1 records
      while (sensorData[i].size() >= track_n ) {
        sensorData[i].erase(sensorData[i].begin());
      }

      vector<double> sfd = sensor_fusion[i];

//      print_coeffs("car :", sfd);

      double x = sfd[1];
      double y = sfd[2];
      double vx = sfd[3];
      double vy = sfd[4];
      double s = sfd[5];
//      double d = sfd[6]; // see above for d

      double tt = 0.04;
      double x1 = x + tt * vx;
      double y1 = y + tt * vy;


      auto sd = getFrenet(x, y, atan2(vy, vx), maps_x, maps_y);
      auto sd1 = getFrenet(x1, y1, atan2(vy, vx), maps_x, maps_y);

      double Dxy = distance(x, y, x1, y1);
      double Dsd = distance(s, d, sd1[0], sd1[1]);

      double vs = (sd1[0] - s) * (Dxy/Dsd) / tt;
      double vd = (sd1[1] - d) * (Dxy/Dsd) / tt;


      sfd.push_back(vs);
      sfd.push_back(vd);

      // Add time
      sfd.push_back(dt_d);
      sensorData[i].push_back(sfd);
    }

//    cout << "sf: cars = " << sensorData.size() << endl;
//    cout << "sf: steps = " << sensorData[0].size() << endl;

  }


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
      history[6].push_back(t); // t
    }


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

    return {SS, DD, TT};

  }

  vector<vector<double> > getTrajXY(int idx, double T = 5.0) {

    auto sd_traj = getTrajSD(idx, T);

    auto xy_traj = getXY(sd_traj[0], sd_traj[1], maps_s, maps_x, maps_y);

    return {xy_traj[0], xy_traj[1], sd_traj[2]};
  }

  // Returns {lane_num, }
  vector<double> estimateLane(int lane, double car_s, double car_d, double car_speed, double end_s, double end_d) {
//    double s_start = car_s - 3 * CAR_LENGTH;

    double s_start = car_s - 20;

    auto closest = getClosestCar(s_start, car_d, lane);

//    double safe_distance = (end_s + 1 * CAR_LENGTH) - s_start;
//    cout << "safe_distance = " << safe_distance << endl;
    double safe_distance = 50;


    if (!closest.empty()) {
      double dist = closest[5] - s_start;
      if (dist < safe_distance) {
        return {}; // not safe to use this lane
      }
      if (dist < 1.5 * safe_distance) {
        return {(double)lane, closest[7]};
      }
    }
    return {(double)lane, SPEED_LIMIT};
  }

  // In a given lane forward from a given position
  vector<double> getClosestCar(double car_s, double car_d, int lane) const {
    double lane_d = LANE_WIDTH * (0.5 + lane);
    double lane_dl = LANE_WIDTH * (lane); // left edge
    double lane_dr = LANE_WIDTH * (1 + lane); // right edge

    double min_s = 100000;
    int min_i = -1;
    for (int i = 0; i < sensorData.size(); ++i) {
      if (sensorData[i].size() > 0) {
        auto last_data = sensorData[i].back();
        double d = last_data[6];
        if (lane_dl < d && d < lane_dr) {
          double s = last_data[5];
          if (s > car_s && s < min_s) {
            min_s = s;
            min_i = i;
          }
        }
      }
    }

    if (min_i < 0) return {};

    return sensorData[min_i].back();


  }

  int size() {return sensorData.size();}


};


Trajectory genTraj(double tLane, double tSpeed, double car_s, double car_d, vector<double> s_start, vector<double> d_start, const SensorFusion& sf, double T = 4.0) {


  auto f_car = sf.getClosestCar(car_s, car_d, tLane);

  double prev_dist = s_start[0] - car_s;
  if (!f_car.empty()) {
    // Have car in a target lane

    double f_car_dist = (f_car[5] - s_start[0]);

    if (f_car_dist < 0) {
      // It is a car between us end of prev path: SLOW DOWN
      tSpeed = min(f_car[7] * 0.5, tSpeed);
      T = 3.0;
    } else if (f_car_dist < 3 * CAR_LENGTH) {
      tSpeed = min(f_car[7] * 0.7, tSpeed);
      T = 3.0;
    } else if (f_car_dist < 5 * CAR_LENGTH) {
      tSpeed = min(f_car[7], tSpeed);
      T = 3.0;
    }
  }

  // Threshold to SPEED_LIMIT (not all cars moving in speed limit at reviewer's simulator :)
  tSpeed = min(tSpeed, SPEED_LIMIT);

  cout << "gen traj for tSpeed = " << tSpeed << endl;

  double end_d = (2.1 + LANE_WIDTH * tLane);

  double delta_v = abs(tSpeed - s_start[1]);

  T = max(delta_v / 4.5, 3.0); // 4.5 m/s per second but not less than 3s

  double avg_v = distance(0.0, 0.0, s_start[1], d_start[1]);
  avg_v = avg_v + distance(0.0, 0.0, tSpeed, 0.0);
  avg_v = avg_v/2;

  double travel_dist = avg_v * T;


  double delta_d = end_d - d_start[0];
  double s_dist = sqrt(travel_dist * travel_dist - delta_d * delta_d);


  // Check for the car and if needed change s_dist AND T


  vector<double> s_end = {s_start[0] + s_dist, tSpeed, 0.0}; // {car_s+100, 15, 0};
  vector<double> d_end = {end_d, 0.0, 0.0};


  auto traj = getJMT(s_start, s_end, d_start, d_end, T);

  return traj;


}


#endif //PATH_PLANNING_HELPERS_H_H
