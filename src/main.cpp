#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
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
#include "test-local.h"
#include "constants.h"





using namespace std;

// for convenience
using json = nlohmann::json;

namespace chrono = std::chrono;



// Still
std::string TEST_JSON_1 = "{\"d\":6.164833,\"end_path_d\":0,\"end_path_s\":0,\"previous_path_x\":[],\"previous_path_y\":[],\"s\":124.8336,\"sensor_fusion\":[[0,870.4026,1132.156,19.77468,0.8812704,85.81837,2.667445],[1,976.1345,1138.357,20.86601,4.658673,191.4412,2.503582],[2,978.3932,1130.807,21.25309,4.735238,191.9534,10.36771],[3,1070.347,1165.639,9.830731,4.096075,288.7551,9.991341],[4,919.1696,1132.875,18.96036,0.1730665,134.5592,2.044153],[5,940.7293,1125.507,19.29597,1.361223,155.0642,10.1185],[6,1054.151,1158.943,2.57131,1.055055,271.3134,9.958789],[7,1097.656,1174.81,11.86194,2.911226,319.1452,9.65999],[8,900.5734,1124.793,19.82975,0.01605316,115.9831,10.00751],[9,874.2359,1128.839,19.28486,-0.08530154,89.6629,5.971819],[10,1047.916,1156.4,0.4504717,0.1831465,264.5796,9.95699],[11,1101.201,1184.121,15.72835,2.854728,324.8331,1.480479]],\"speed\":0,\"x\":909.48,\"y\":1128.67,\"yaw\":0}";


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Handler State
  HandlerState hState;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }





  h.onMessage([&hState,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

//      cout << "s = " << s << endl;

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

//          cout << "j[1] = " << j[1] << endl;

        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];



          int maxPoints = 50;


          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          cout << "----- " << hState.cnt << " ------------------" << endl;

          // Tick for dt and other params
          hState.tick();

          // Speed in m/s
          double car_speed_m = 1609.344 * car_speed / 3600.0;


          hState.save(car_x, car_y, car_yaw, car_s, car_d);



          // Check do we have all info
          if (!hState.ready) {
            hState.prev_v = car_speed_m;

            // Send empty
            cout << "Not READY yet!" << endl;
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
            auto msg = "42[\"control\","+ msgJson.dump()+"]";
            ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            return;
          }

          hState.acc = (car_speed_m - hState.prev_v) / hState.dt;
          hState.prev_v = car_speed_m;


          // We are ready with DT
          cout << "DT = " << hState.dt << endl;
          cout << "x,y = " << car_x << ", " << car_y << endl;
          cout << "car_speed = " << car_speed << "( " << car_speed_m << " )" << endl;
          cout << "s,d = " << car_s << ", " << car_d << endl;
          cout << "end_path s,d = " << end_path_s << ", " << end_path_d << endl;
          cout << "prev_path.size = " << previous_path_x.size() << endl;
          cout << "acc = " << hState.acc << endl;


          // Get start_s and start_d params
            // Check prev_path
            // Check prev_traj

          vector<double> s_start = {car_s, car_speed_m, 0};
          vector<double> s_end;

          vector<double> d_start = {car_d, 0, 0};
          vector<double> d_end;


          int forwardN = 0;

          double curr_time;

          if (hState.path) {

            forwardN = 5;

            if (previous_path_x.size() <= forwardN) {
              forwardN = previous_path_x.size() - 1;
            }
            double forward_x = previous_path_x[forwardN];
            double forward_y = previous_path_y[forwardN];


            curr_time = findTimeInTrajByXY(hState.prev_traj, car_x, car_y, car_yaw, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            cout << "found curr_time = " << curr_time << endl;

            double forward_time;
            forward_time = findTimeInTrajByXY(hState.prev_traj, forward_x, forward_y, car_yaw, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            cout << "found forward_time = " << forward_time << endl;

            // for testing - TODO: fix names
//            curr_time = forward_time;
//            cout << "Curr Time == Forward Time" << endl;
            curr_time = hState.prev_traj.T;
            cout << "Curr Time == T == " << hState.prev_traj.T << endl;


            // Look at s,d params at the curr_time
            double curr_s, curr_s_v, curr_s_a;
            double curr_d, curr_d_v, curr_d_a;
            curr_s = poly_calc(hState.prev_traj.s_coeffs, curr_time, 0);
            curr_s_v = poly_calc(hState.prev_traj.s_coeffs, curr_time, 1);
            curr_s_a = poly_calc(hState.prev_traj.s_coeffs, curr_time, 2);
            print_coeffs("curr_s = ", {curr_s, curr_s_v, curr_s_a});

            curr_d = poly_calc(hState.prev_traj.d_coeffs, curr_time, 0);
            curr_d_v = poly_calc(hState.prev_traj.d_coeffs, curr_time, 1);
            curr_d_a = poly_calc(hState.prev_traj.d_coeffs, curr_time, 2);
            print_coeffs("curr_d = ", {curr_d, curr_d_v, curr_d_a});

//            s_start[0] = curr_s;
            s_start[0] = end_path_s;

            s_start[1] = curr_s_v;
            s_start[2] = curr_s_a;

//            d_start[0] = curr_d;
            d_start[0] = end_path_d;

            d_start[1] = curr_d_v;
            d_start[2] = curr_d_a;
          }

//          cout << "s_start[0] = " << s_start[0] << endl;
//          cout << "d_start[0] = " << d_start[0] << endl;


          print_coeffs("FINAL s_start = ", s_start);
          print_coeffs("FINAL d_start = ", d_start);


          // Make a straight line trajectory
//          s_start = {car_s, car_speed, 0};
          s_end = {s_start[0] + 100, 15, 0};

//          d_start = {car_d, 0, 0};
          d_end = {6.0, 0, 0};

          double T = 8.0;

//          double s_avg_v = (s_start[1] + s_end[1]) / 2;
//          double T = (s_end[0] - s_start[0]) / s_avg_v;
//          T = 8.0;


          // Mimic prev trajectory
//          if (hState.path) {
//            double last_t = hState.prev_traj.T;
//            s_end[0] = poly_calc(hState.prev_traj.s_coeffs, last_t);
//            d_end[0] = poly_calc(hState.prev_traj.d_coeffs, last_t);
//            T = hState.prev_traj.T - curr_time;
//          }


          print_coeffs("FINAL s_end = ", s_end);
          print_coeffs("FINAL d_end = ", d_end);
          cout << "FINAL T = " << T << endl;


          Trajectory traj = getJMT(s_start, s_end, d_start, d_end, T);


          auto acc_stats = traj_stats_acc(traj);
          cout << "acc_per_sec = " << acc_stats[0] << ", max_acc = " << acc_stats[1] << endl;

//          cout << "TRAJ: " << traj.str() << endl;


          vector<vector<double>> xy;


          /* Commented reusing old path


          if (hState.path) {
            xy = getXYPathFromTraj(hState.prev_traj, map_waypoints_s, map_waypoints_x, map_waypoints_y, curr_time);

            add_to_log(hState.dt, car_x, car_y, car_yaw, car_s, car_d, car_speed_m, hState.prev_traj, previous_path_x, previous_path_y);

          } else {
            xy = getXYPathFromTraj(traj, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            add_to_log(hState.dt, car_x, car_y, car_yaw, car_s, car_d, car_speed_m, traj, previous_path_x, previous_path_y);

            hState.path = true;
            hState.prev_traj = traj;

          }

          next_x_vals = xy[0];
          next_y_vals = xy[1];
          */


          if (hState.path) {

            // simply copy previous
//            for (int i = 0; i < previous_path_x.size(); ++i) {
//              next_x_vals.push_back(previous_path_x[i]);
//              next_y_vals.push_back(previous_path_y[i]);
//            }


            if (previous_path_x.size() >= maxPoints) {

              // copy previous
              for (int i = 0; i < previous_path_x.size(); ++i) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }

              add_to_log(hState.dt, car_x, car_y, car_yaw, car_s, car_d, car_speed_m, hState.prev_traj, previous_path_x, previous_path_y, end_path_s, end_path_d);

            } else {
              // Less than maxPoints


              // copy previous
              for (int i = 0; i < previous_path_x.size(); ++i) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }

              // and add new one
//              xy = getXYPathFromTraj(traj, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              xy = getXYPathConnected(5, previous_path_x, previous_path_y, traj, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              for (int i = 1; i < xy[0].size(); ++i) {
                next_x_vals.push_back(xy[0][i]);
                next_y_vals.push_back(xy[1][i]);
              }

              hState.path = true;
              hState.prev_traj = traj;

              add_to_log(hState.dt, car_x, car_y, car_yaw, car_s, car_d, car_speed_m, traj, previous_path_x, previous_path_y, end_path_s, end_path_d);


            }


          } else {
            xy = getXYPathFromTraj(traj, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals = xy[0];
            next_y_vals = xy[1];

            add_to_log(hState.dt, car_x, car_y, car_yaw, car_s, car_d, car_speed_m, traj, previous_path_x, previous_path_y, end_path_s, end_path_d);

            hState.path = true;
            hState.prev_traj = traj;

          }

          /*



          // forwardN logic
          if (forwardN > 0) {

            auto traj_data = getSDbyTraj(traj, PATH_TIMESTEP); // s,d,t
            cout << "traj_data.size = " << traj_data[0].size() << endl;
            cout << "traj_data[0] = " << traj_data[2][0] << ", " << traj_data[2][traj_data[2].size()-1] << endl;

            auto xy_traj = getXY(traj_data[0], traj_data[1], map_waypoints_s, map_waypoints_x, map_waypoints_y);

            vector<double> xx_n;
            vector<double> yy_n;
            vector<double> tt_n;


            // Add one first point
//            tt_n.push_back(- PATH_TIMESTEP * forwardN);
//            xx_n.push_back(previous_path_x[0]);
//            yy_n.push_back(previous_path_y[0]);

            // Add first forwardN steps of previous_path
            for (int i = forwardN; i > 0; --i) {
              tt_n.push_back(- PATH_TIMESTEP * i);
              xx_n.push_back(previous_path_x[forwardN - i]);
              yy_n.push_back(previous_path_y[forwardN - i]);
            }


            // Add points from new traj
            for (int i = 0; i < traj_data[0].size(); ++i) {
              tt_n.push_back(traj_data[2][i]);
              xx_n.push_back(xy_traj[0][i]);
              yy_n.push_back(xy_traj[1][i]);
            }

//            next_x_vals = xy[0];
//            next_y_vals = xy[1];


            //double T = ss.size() * timestep;
            double T = traj.T;

//            cout << "xx_n, yy_n" << endl;
//            print_vals(xx_n, yy_n, 100);


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

            for (int i = 0; i < XX_smooth.size(); ++i) {
              next_x_vals.push_back(XX_smooth[i]);
              next_y_vals.push_back(YY_smooth[i]);
            }



            // Show prev values
//            vector<double> prev_x;
//            vector<double> prev_y;
//            for (int i = 0; i < previous_path_x.size(); ++i) {
//              prev_x.push_back(previous_path_x[i]);
//              prev_y.push_back(previous_path_y[i]);
//            }
//            print_vals(prev_x, prev_y, 15);
//
//            cout << "mix up" << endl;
//            print_vals(next_x_vals, next_y_vals, 15);


          } else {
            xy = getXYPathFromTraj(traj, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            next_x_vals = xy[0];
            next_y_vals = xy[1];

//            cout << "no mix up";
//            print_vals(next_x_vals, next_y_vals, 15);
          }


          add_to_log(hState.dt, car_x, car_y, car_yaw, car_s, car_d, car_speed_m, traj, previous_path_x, previous_path_y);

          hState.path = true;
          hState.prev_traj = traj;
          */



          /*

          if (hState.path) {
            // Analyze prev traj and found curr_time
            double curr_time;
            curr_time = findTimeInTrajByXY(hState.prev_traj, car_x, car_y, car_yaw, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            cout << "found curr_time = " << curr_time << endl;

            double f = fmod(curr_time, PATH_TIMESTEP);
            cout << "f = " << f << endl;
            cout << "r = " << curr_time - f << endl;
            double timeShift = PATH_TIMESTEP - f;
            double timePrev = curr_time - f;

            cout << "timeShift = " << timeShift << endl;
            cout << "timePrev = " << timePrev << endl;

            auto xy_prev = getXYPathFromTraj(hState.prev_traj, map_waypoints_s, map_waypoints_x, map_waypoints_y, 0.0);

            vector<double> tt = getTT(PATH_TIMESTEP, previous_path_x.size());

            // Spline Smoothing XY line
            tk::spline splX;
            splX.set_points(tt, previous_path_x);

            tk::spline splY;
            splY.set_points(tt, previous_path_y);


            cout << "set prev traj ..." << endl;

            unsigned long N = 100;
            if (previous_path_x.size() > N)
              N = previous_path_x.size();

            for (int i = 0; i < N; ++i) {
              next_x_vals.push_back(splX(i * PATH_TIMESTEP));
              next_y_vals.push_back(splY(i * PATH_TIMESTEP));
            }

//            next_x_vals = xy_prev[0];
//            next_y_vals = xy_prev[1];

          } else {

            hState.path = true;
            hState.prev_traj = traj;

            next_x_vals = xy[0];
            next_y_vals = xy[1];

//            for (int i = 0; i < xy[0].size(); ++i) {
//              cout << xy[2][i] << "\t" << xy[0][i] << "\t" << xy[1][i] << endl;
//            }
          }
           */

//          json x_j = json(xy[0]);
//          json y_j = json(xy[1]);






          /*
          if (previous_path_x.size() > 10) {



            // Reconstruct previous path spline
            vector<double> prevTT;
            prevTT = getTT(PATH_TIMESTEP, previous_path_x.size());

            // Spline Smoothing XY line
            tk::spline splPrevX;
            splPrevX.set_points(prevTT, previous_path_x);

            tk::spline splPrevY;
            splPrevY.set_points(prevTT, previous_path_y);

            // Combine new path with previous_path
            auto N = xy[0].size();
            auto prev_N = previous_path_x.size();
//            if (prev_N > 10) prev_N = 10;
            if (prev_N > N) prev_N = N;
            cout << "mix with prev: prev_N = " << prev_N << ", N = " << N << endl;
            for (int i = 0; i < N; i++) {
              if (i < prev_N) {
                // mix
//                double prev_ratio = 1 - i / prev_N;
                double prev_ratio = 1;
                next_x_vals.push_back(prev_ratio * splPrevX(i * PATH_TIMESTEP) + (1 - prev_ratio) * xy[0][i]);
                next_y_vals.push_back(prev_ratio * splPrevY(i * PATH_TIMESTEP) + (1 - prev_ratio) * xy[1][i]);
              } else {
                // put as is
                next_x_vals.push_back(xy[0][i]);
                next_y_vals.push_back(xy[1][i]);
              }

            }
          } else {
            cout << "no previous path so no mixing" << endl;
            // Just set up new path to next_vals
            next_x_vals = xy[0];
            next_y_vals = xy[1];
          }
           */

//          print_vals(next_x_vals, next_y_vals);




          /*

          // Transform from s,d to x,y
          if (!hState.path) {
            cout << "path = ";



            auto xy = getXYPath(traj_data[0], traj_data[1], TRAJ_TIMESTEP * 2, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            next_x_vals = xy[0];
            next_y_vals = xy[1];

            print_vals(next_x_vals, next_y_vals);

            hState.path = true;
            hState.prev_traj = traj;
          } else {

            // Analyze prev traj and found curr_time
            double curr_time;
            curr_time = findTimeInTrajByXY(hState.prev_traj, car_x, car_y, car_yaw, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            cout << "found curr_time = " << curr_time << endl;

            // Look at s,d params at the curr_time
            double curr_s, curr_s_v, curr_s_a;
            double curr_d, curr_d_v, curr_d_a;
            curr_s = poly_calc(hState.prev_traj.s_coeffs, curr_time, 0);
            curr_s_v = poly_calc(hState.prev_traj.s_coeffs, curr_time, 1);
            curr_s_a = poly_calc(hState.prev_traj.s_coeffs, curr_time, 2);
            print_coeffs("curr_s = ", {curr_s, curr_s_v, curr_s_a});

            curr_d = poly_calc(hState.prev_traj.d_coeffs, curr_time, 0);
            curr_d_v = poly_calc(hState.prev_traj.d_coeffs, curr_time, 1);
            curr_d_a = poly_calc(hState.prev_traj.d_coeffs, curr_time, 2);
            print_coeffs("curr_d = ", {curr_d, curr_d_v, curr_d_a});



            vector<double> prevTT;
            prevTT = getTT(PATH_TIMESTEP, previous_path_x.size());

            // Spline Smoothing XY line
            tk::spline splPrevX;
            splPrevX.set_points(prevTT, previous_path_x);

            tk::spline splPrevY;
            splPrevY.set_points(prevTT, previous_path_y);

            double prev_speed_x = splPrevX.deriv(1, 0.0);
            double prev_speed_y = splPrevY.deriv(1, 0.0);

            cout << "prev_speed = " << prev_speed_x << ", " << prev_speed_y << endl;
            cout << "prev_speed_v = " << distance(0,0,prev_speed_x,prev_speed_y) << endl;

            // copy previous path as a whole
            for (int i = 0; i < previous_path_x.size(); ++i) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

//            print_vals(next_x_vals, next_y_vals);


          }

          */

          if (!next_x_vals.empty()) {
            cout << "next_vals.size = " << next_x_vals.size() << " from {"
                 << next_x_vals[0] << ", " << next_y_vals[0] << "} to {"
                 << next_x_vals[next_x_vals.size()-1] << ", " << next_y_vals[next_y_vals.size()-1]
                 << "}" << endl;
          } else {
            cout << "next_vals.size = " << next_x_vals.size() << endl;
          }



          /*
          double dist_inc1 = 0.5/50;
          double dist = 0;
          for (int i = 0; i < 50; i++) {
            dist += dist_inc1 * (i+1);
            next_x_vals.push_back(car_x + (dist) * cos(deg2rad(car_yaw)));
            next_y_vals.push_back(car_y + (dist) * sin(deg2rad(car_yaw)));
          }
          */

          // Test sensor fusion
          // Sort/Comp https://stackoverflow.com/questions/33046173/sorting-json-values-alphabetically-c

          // Typeid: https://stackoverflow.com/questions/81870/is-it-possible-to-print-a-variables-type-in-standard-c
//          cout << "decltype = " << typeid(sensor_fusion).name() << endl;
//
//          cout << "sensor_fusion:" << endl;
//          for (int i = 0; i < sensor_fusion.size(); ++i) {
//            cout << "[" << sensor_fusion[i][0] << "] = "
//                 << sensor_fusion[i][5] << ", " << sensor_fusion[i][6] << endl;
//          }


          // cout << "sensor fusion = " << sensor_fusion.dump(2) << endl;

          //cout << "pos = " << car_x << ", " << car_y << ", " << car_yaw << endl;
//          cout << "s,d = " << car_s << ", " << car_d << endl;


          /*
          double pos_x;
          double pos_y;
          double angle;
          int path_size = previous_path_x.size();

          cout << "path_size = " << path_size << endl;

          for (int i = 0; i < path_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          if (path_size == 0) {
            pos_x = car_x;
            pos_y = car_y;
            angle = deg2rad(car_yaw);
          } else {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];
            double pos_x2 = previous_path_x[path_size-2];
            double pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y - pos_y2, pos_x - pos_x2);

          }

          double dist_inc = 0.5;
          for (int i = 0; i < 50 - path_size; i++) {
            next_x_vals.push_back(pos_x + (dist_inc) * cos(angle + (i+1)*pi()/100));
            next_y_vals.push_back(pos_y + (dist_inc) * sin(angle + (i+1)*pi()/100));
            pos_x += (dist_inc) * cos(angle + (i+1)*pi()/100);
            pos_y += (dist_inc) * sin(angle + (i+1)*pi()/100);
          }
          */

          /*
          TODO:
            1) Accelerate to the 25m/s in the same line and drive on one speed
            in the line.
            2) Follow the vehicle in this line. (keep distance)
            3) Define states and FSM.
            4) Generate traj for different states.
            5) Look at spline.
          */


          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
                         char *message, size_t length) {
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });

  if (LOCAL) {
    // Test on local data
//    testLocal(TEST_JSON_1, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//    testLocalStored(map_waypoints_s, map_waypoints_x, map_waypoints_y);
    testLocalTrajectories(map_waypoints_s, map_waypoints_x, map_waypoints_y);
  } else {
    prepare_log();

    int port = 4567;
    if (h.listen(port)) {
      std::cout << "Listening to port " << port << std::endl;
    } else {
      std::cerr << "Failed to listen to port" << std::endl;
      return -1;
    }
    h.run();

  }
}
