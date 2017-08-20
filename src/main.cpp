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
//#include "matplotlibcpp.h"
#include <typeinfo>
#include <unistd.h>
#include "spline.h"
#include "helpers.h"
//#include "test-local.h"
#include "constants.h"





using namespace std;

// for convenience
using json = nlohmann::json;

namespace chrono = std::chrono;


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Handler State - saves car state between onMessage calls
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


  // Incapsulates sensor fusion info and functions of prediction
  SensorFusion sf(map_waypoints_s, map_waypoints_x, map_waypoints_y);

  int targetLane = 1;


  h.onMessage([&targetLane,&sf,&hState,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {

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

          double end_path_x = 0.0;
          double end_path_y = 0.0;
          double end_path_yaw = car_yaw;

          if (previous_path_x.size() > 0) {
            int last_idx = previous_path_x.size() - 1;
            end_path_x = previous_path_x[last_idx];
            end_path_y = previous_path_y[last_idx];
            end_path_yaw = atan2(previous_path_y[last_idx] - previous_path_y[last_idx-1], previous_path_x[last_idx] - previous_path_x[last_idx-1]);
          }

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];




          int maxPoints = MAX_POINTS;
          double targetSpeed = SPEED_LIMIT; //49.5 * 1609.344 / 3600.0;





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
          /*
          if (!hState.ready) {
            hState.prev_v = car_speed_m;

            // Send empty
            cout << "Not READY yet!" << endl;
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
            auto msg = "42[\"control\","+ msgJson.dump()+"]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            return;
          }
           */

          double dt_d = 0.0;
          if (previous_path_x.size() > 0) {
            dt_d = PATH_TIMESTEP * (maxPoints - previous_path_x.size());
          }

          /*
          hState.acc = (car_speed_m - hState.prev_v) / hState.dt;
          hState.prev_v = car_speed_m;
           */


          // We are ready with DT
          cout << "DT = " << hState.dt << " ( " << dt_d << " )" << endl;
          cout << "x,y = " << car_x << ", " << car_y << endl;
          cout << "car_speed = " << car_speed << "( " << car_speed_m << ", target = " << targetSpeed << " )" << endl;
          cout << "s,d = " << car_s << ", " << car_d << endl;
          cout << "end_path s,d = " << end_path_s << ", " << end_path_d << endl;
//          cout << "end_path x,y,yaw = " << end_path_x << ", " << end_path_y << ", " << end_path_yaw << endl;
          cout << "prev_path.size = " << previous_path_x.size() << endl;
//          cout << "acc = " << hState.acc << endl;


          // Add data to sensor fusion
          sf.add(sensor_fusion, dt_d);


          // Set initial start_s/d params
          vector<double> s_start = {car_s, car_speed_m, 0};
          vector<double> s_end;

          vector<double> d_start = {car_d, 0, 0};
          vector<double> d_end;


          double curr_time;

          if (hState.path) {
            // We do have previous trajectory that we've processed


            // were we are on that trajectory (end of path, actually)
            curr_time = findTimeInTrajByXY(hState.prev_traj, end_path_x, end_path_y, end_path_yaw, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            cout << "found curr_time = " << curr_time << endl;


            // Look at s,d params at the curr_time
            double curr_s, curr_s_v, curr_s_a;
            double curr_d, curr_d_v, curr_d_a;
            curr_s = poly_calc(hState.prev_traj.s_coeffs, curr_time, 0);
            curr_s_v = poly_calc(hState.prev_traj.s_coeffs, curr_time, 1);
            curr_s_a = poly_calc(hState.prev_traj.s_coeffs, curr_time, 2);

            curr_d = poly_calc(hState.prev_traj.d_coeffs, curr_time, 0);
            curr_d_v = poly_calc(hState.prev_traj.d_coeffs, curr_time, 1);
            curr_d_a = poly_calc(hState.prev_traj.d_coeffs, curr_time, 2);

            // Due to the nature of non-linear transform between xy coord and sd frenet coord system
            // we need to smooth the predicted end path sd coords. (EXPERIMENTAL)
            double alpha = 0.5; // so just curr_s/d looks good at the end :)
            s_start[0] = end_path_s * alpha + curr_s * (1 - alpha);

            s_start[1] = curr_s_v;
            s_start[2] = curr_s_a;

            d_start[0] = end_path_d * alpha + curr_d * (1 - alpha);

            d_start[1] = curr_d_v;
            d_start[2] = curr_d_a;
          }

          print_coeffs("FINAL s_start = ", s_start);
          print_coeffs("FINAL d_start = ", d_start);



          // List of adjacent lanes that we can test for better results
          vector<int> adj_lanes;
          if (targetLane == 0) {
            adj_lanes.push_back(1);
          } else if (targetLane == 1) {
            adj_lanes.push_back(0);
            adj_lanes.push_back(2);
          } else if (targetLane == 2) {
            adj_lanes.push_back(1);
          }


          // Check alt lanes and change taregetLane, targetSpeed if needed
          if (hState.path) {
            // Get current planned speed
            double planned_speed = poly_calc(hState.prev_traj.s_coeffs, hState.prev_traj.T, 1);
//            cout << "planned speed = " << planned_speed << endl;

            for (int i = 0; i < adj_lanes.size(); ++i) {
              int lc = adj_lanes[i]; // line candidate

              // Estimate the possibility of change to that line with the speed param
              auto lc_data = sf.estimateLane(lc, car_s, car_d, car_speed_m, s_start[0], d_start[0]);
//              cout << "eval lane L" << lc << endl;
//              print_coeffs("lc_data = ", lc_data);


              // Suggested line is safe to use, so if we will gain at least 3 m/s speed let's use this suggestion
              if (!lc_data.empty() && lc_data[1] > planned_speed + 3) {
//                cout << "select line candidate " << lc << endl;
                targetLane = lc;
                planned_speed = lc_data[1];
                targetSpeed = planned_speed;
              }
            }

          }

          cout << "TARGET: Lane = " << targetLane << ", " << targetSpeed << endl;


          // Set loop_ends flag if we are crossing the 'end of the world'
          // so to not use the broken trajectory while we are back on normal coordinates
          bool loop_ends = false;
          if (car_s > s_start[0] || s_start[0] - car_s > 1.5 * MAX_POINTS * PATH_TIMESTEP * SPEED_LIMIT) {
            cout << "LOOP ENDS!!!!!!" << endl;
            loop_ends = true;
            s_start[0] = end_path_s;
            d_start[0] = end_path_d;
          }

          // Generate trajectory for the goal: tartgetLane + targetSpeed
          auto traj = genTraj(targetLane, targetSpeed, car_s, car_d, s_start, d_start, sf);


          // Trajectory analysis
          auto acc_stats = traj_stats_acc(traj);
          auto j_stats = traj_stats_jerk(traj);
//          cout << "acc_per_sec = " << acc_stats[0] << ", max_acc = " << acc_stats[1] << endl;
//          cout << "jerk_per_sec = " << j_stats[0] << ", max_jerk = " << j_stats[1] << endl;


          vector<vector<double>> xy;


          // Set up the next points based on the prev points
          if (hState.path) {

            // If our previous path already contains maxPonts just use them
            if (previous_path_x.size() >= maxPoints) {


              // copy previous
              for (int i = 0; i < previous_path_x.size(); ++i) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }

              /* DEBUG: save state to file for later local use
              add_to_log(hState.dt, dt_d, car_x, car_y, car_yaw, car_s, car_d, car_speed_m, hState.prev_traj, hState.prev_xy[0], hState.prev_xy[1], hState.prev_next_idx, previous_path_x, previous_path_y, end_path_s, end_path_d, next_x_vals, next_y_vals, sensor_fusion);
               */

            } else {
              // If it's less than MAX_POINTS


              // first copy previous points
              for (int i = 0; i < previous_path_x.size(); ++i) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }

              bool have_stash = hState.prev_next_idx < hState.prev_xy[0].size();

              double gamble = (double) rand() / RAND_MAX;
//              cout << "gamble = " << gamble << endl;

              // second check existing path points in stash - hState.prev_xy
              if (have_stash && loop_ends) {


                // just copy points from stash (using during the lock up period while crossing the "end of the world"

                int prev_next_idx = hState.prev_next_idx;

                // Have points copy up to maxPoints (or what we have)
                for (int i = 0; i < maxPoints - previous_path_x.size() && i + hState.prev_next_idx < hState.prev_xy[0].size(); ++i) {
                  next_x_vals.push_back(hState.prev_xy[0][hState.prev_next_idx + i]);
                  next_y_vals.push_back(hState.prev_xy[1][hState.prev_next_idx + i]);
                  ++prev_next_idx;
                }

                hState.prev_next_idx = prev_next_idx;


                cout << "stash total = " << hState.prev_xy[0].size() << endl;
                cout << "stash idx = " << hState.prev_next_idx << endl;
                cout << "stash traj = " << hState.prev_traj.str() << endl;

                /* DEBUG: save state to file for later local use
                add_to_log(hState.dt, dt_d, car_x, car_y, car_yaw, car_s, car_d, car_speed_m, hState.prev_traj, hState.prev_xy[0], hState.prev_xy[1], hState.prev_next_idx, previous_path_x, previous_path_y, end_path_s, end_path_d, next_x_vals, next_y_vals, sensor_fusion);
                */

              } else {
                // Don't have points on stash or can use new trajectory - use it

                // Use new trajectory


                xy = getXYPathConnected1(previous_path_x, previous_path_y, car_x, car_y, car_yaw, traj, map_waypoints_s, map_waypoints_x, map_waypoints_y);

                int path_next_idx = 1;

                for (int i = 0; i < maxPoints - previous_path_x.size() && i < xy[0].size(); ++i) {
                  next_x_vals.push_back(xy[0][i+1]);
                  next_y_vals.push_back(xy[1][i+1]);
                  ++path_next_idx;
                }

                hState.path = true;
                hState.prev_traj = traj;
                hState.prev_xy = xy;
                hState.prev_next_idx = path_next_idx;

                /* DEBUG: save state to file for later local use
                add_to_log(hState.dt, dt_d, car_x, car_y, car_yaw, car_s, car_d, car_speed_m, traj, hState.prev_xy[0], hState.prev_xy[1], hState.prev_next_idx, previous_path_x, previous_path_y, end_path_s, end_path_d, next_x_vals, next_y_vals, sensor_fusion);
                */


              }



            } // end of << if (hState.path) {


          } else {
            // First trajectory

            xy = getXYPathFromTraj(traj, map_waypoints_s, map_waypoints_x, map_waypoints_y);

//            xy = getXYPathConnected1(previous_path_x, previous_path_y, car_x, car_y, car_yaw, traj, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            int path_next_idx = 0;

            // Copy to maxPoints elements in prev points
            for (int i = 0; i < maxPoints - previous_path_x.size() && i < xy[0].size(); ++i) {
              next_x_vals.push_back(xy[0][i]);
              next_y_vals.push_back(xy[1][i]);
              ++path_next_idx;
            }

            hState.path = true;
            hState.prev_traj = traj;
            hState.prev_xy = xy;
            hState.prev_next_idx = path_next_idx;

            /* DEBUG: save state to file for later local use
            add_to_log(hState.dt, dt_d, car_x, car_y, car_yaw, car_s, car_d, car_speed_m, traj, hState.prev_xy[0], hState.prev_xy[1], hState.prev_next_idx, previous_path_x, previous_path_y, end_path_s, end_path_d, next_x_vals, next_y_vals, sensor_fusion);
            */



          }


          // Just log for DEBUG
          if (!next_x_vals.empty()) {
            cout << "next_vals.size = " << next_x_vals.size() << " from {"
                 << next_x_vals[0] << ", " << next_y_vals[0] << "} to {"
                 << next_x_vals[next_x_vals.size()-1] << ", " << next_y_vals[next_y_vals.size()-1]
                 << "}";

            // Check max speed in the resulting next_vals
            double max_dist = 0;
            for (int i = 1; i < next_x_vals.size(); ++i) {
              double dist = distance(next_x_vals[i-1], next_y_vals[i-1], next_x_vals[i], next_y_vals[i]);
              if (dist > max_dist) {
                max_dist = dist;
              }
            }
            cout << ", ms = " << (max_dist/0.02) << endl;


          } else {
            cout << "next_vals.size = " << next_x_vals.size() << endl;
          }


          // Set up next path points
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          this_thread::sleep_for(chrono::milliseconds(200));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  /* DEDUG purpose
  if (LOCAL) {
//    testLocal(TEST_JSON_1, map_waypoints_s, map_waypoints_x, map_waypoints_y);
//    testLocalStored(map_waypoints_s, map_waypoints_x, map_waypoints_y);
    testLocalTrajectories(map_waypoints_s, map_waypoints_x, map_waypoints_y);
//    testTrajectoriesGen(map_waypoints_s, map_waypoints_x, map_waypoints_y);
  } else {
    prepare_log();
  */

    int port = 4567;
    if (h.listen(port)) {
      std::cout << "Listening to port " << port << std::endl;
    } else {
      std::cerr << "Failed to listen to port" << std::endl;
      return -1;
    }
    h.run();

  //} << DEBUG
}
