//
// Created by Pavlo Bashmakov on 8/3/17.
//

#ifndef PATH_PLANNING_HELPERS_H_H
#define PATH_PLANNING_HELPERS_H_H


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
//#include "matplotlibcpp.h"
//#include <typeinfo>
#include <unistd.h>
#include "spline.h"
#include "helpers.h"

using namespace std;


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


vector<vector<double> > getXYPath(vector<double> ss, vector<double> dd, double timestep, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y) {

  auto xy = getXY(ss, dd, maps_s, maps_x, maps_y);

//            next_x_vals = xy[0];
//            next_y_vals = xy[1];


  double T = ss.size() * timestep;

  vector<double> TT;
  for (int i = 0; i < ss.size(); ++i) {
    TT.push_back((double)i * timestep);
  }

  // Spline Smoothing XY line
  tk::spline splX;
  splX.set_points(TT, xy[0]);

  tk::spline splY;
  splY.set_points(TT, xy[1]);

  vector<double> XX_smooth;
  vector<double> YY_smooth;
  double t = 0.0;
  double timestep2 = 0.02; // path requirements
  while (t <= T) {
    XX_smooth.push_back(splX(t));
    YY_smooth.push_back(splY(t));
    t += timestep2;
  }

  return {XX_smooth, YY_smooth};

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


void print_vals(vector<double> X, vector<double> Y, int max_num = 10) {
  int m = min(max_num, (int)X.size());
  for (int i = 0; i < m; ++i) {
    cout << "[" << i << "] = " << X[i] << ", " << Y[i] << endl;
  }
  if (m < X.size()) {
    cout << ". . . ." << endl;
  }
}



#endif //PATH_PLANNING_HELPERS_H_H
