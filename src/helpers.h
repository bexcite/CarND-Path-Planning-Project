//
// Created by Pavlo Bashmakov on 8/3/17.
//

#ifndef PATH_PLANNING_HELPERS_H_H
#define PATH_PLANNING_HELPERS_H_H


#include <fstream>
#include <math.h>
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

  cout << "smooth_TT[0] = " << TT[0] << ", smooth_TT[end] = " << TT[TT.size()-1] << endl;

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
  while (t <= traj.T /* + timestep*/) {
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
  cout << "traj_data.size = " << traj_data[0].size() << endl;
  cout << "traj_data[0] = " << traj_data[2][0] << ", " << traj_data[2][traj_data[2].size()-1] << endl;

  auto xy = getXYPath(traj_data[0], traj_data[1], traj_data[2], maps_s, maps_x, maps_y);

  double t = 0.0;
  while (t < timeShift) {
    xy[0].erase(xy[0].begin());
    xy[1].erase(xy[1].begin());
    t += PATH_TIMESTEP;
  }

  return xy;
}



void prepare_log() {
  ofstream logFile ("log.out", ios::out);
  if (logFile.is_open()) {
    //logFile << "--- start ----" << endl;
    logFile.close();
  }
}



void add_to_log(double dt, double car_x, double car_y, double car_yaw, double car_s, double car_d, double car_speed, Trajectory traj, json prev_x, json prev_y) {
  ofstream logFile ("log.out", ios::out | ios::app);
  if (logFile.is_open()) {
    json j;
    j["dt"] = dt;
    j["car_x"] = car_x;
    j["car_y"] = car_y;
    j["car_yaw"] = car_yaw;
    j["car_s"] = car_s;
    j["car_d"] = car_d;
    j["car_speed"] = car_speed; // m/s
    j["s_coeffs"] = traj.s_coeffs;
    j["d_coeffs"] = traj.d_coeffs;
    j["traj_t"] = traj.T;
    j["prev_x"] = prev_x;
    j["prev_y"] = prev_y;
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
    if (a > max_acc) {
      max_acc = a;
    }
    total_acc += abs(a * PATH_TIMESTEP);
  }
  double acc_per_second = total_acc / traj.T;

  return {acc_per_second, max_acc};


}



#endif //PATH_PLANNING_HELPERS_H_H
