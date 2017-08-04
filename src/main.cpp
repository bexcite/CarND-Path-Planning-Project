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

int LOCAL = 0;

double PATH_TIMESTEP = 0.02; // 50 Hz - rate for final path
double TRAJ_TIMESTEP = 0.2; // 5 Hz - rate for calculating trajectory

using namespace std;

// for convenience
using json = nlohmann::json;

namespace chrono = std::chrono;


/* ==== HandlerState ==== */
class HandlerState {
public:

  HandlerState() {
    prev_clk = chrono::high_resolution_clock::now();
    initialized = false;
  }

  virtual ~HandlerState() {}

  chrono::time_point<chrono::high_resolution_clock> prev_clk;

  bool initialized = false;

  bool ready = false;

  bool path = false;

  double dt;

  void tick() {

    // Calc dt time
    auto clk = chrono::high_resolution_clock::now();
    dt = chrono::duration<double>(clk - prev_clk).count();
    prev_clk = clk;

    if (initialized && !ready) {
      ready = true;
    }
    if (!initialized) {
      initialized = true;
    }
  }

};


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
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];


          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // Tick for dt and other params
          hState.tick();

          // Check do we have all info
          if (!hState.ready) {
            // Send empty
            cout << "Not READY yet!" << endl;
            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;
            auto msg = "42[\"control\","+ msgJson.dump()+"]";
            ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            return;
          }

          // We are ready with DT
          cout << "DT = " << hState.dt << endl;
          cout << "x,y = " << car_x << ", " << car_y << endl;
          cout << "car_speed = " << car_speed << endl;
          cout << "s,d = " << car_s << ", " << car_d << endl;
          cout << "end_path s,d = " << end_path_s << ", " << end_path_d << endl;
          cout << "prev_path.size = " << previous_path_x.size() << endl;

          // Make a straight line trajectory
          vector<double> s_start = {car_s, car_speed, 0};
          vector<double> s_end = {car_s+100, 20, 0};

          vector<double> d_start = {car_d, 0, 0};
          vector<double> d_end = {car_d, 0, 0};

          double T = 8.0;

          auto s_coeffs = JMT(s_start, s_end, T);
          auto d_coeffs = JMT(d_start, d_end, T);

          double timestep = TRAJ_TIMESTEP;
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


          // Transform from s,d to x,y
          if (!hState.path) {
            cout << "path = ";

            vector< vector<double> > xy;

            xy = getXYPath(SS, DD, TRAJ_TIMESTEP, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            next_x_vals = xy[0];
            next_y_vals = xy[1];

            print_vals(next_x_vals, next_y_vals);

            /*
            xy = getXY(SS, DD, map_waypoints_s, map_waypoints_x, map_waypoints_y);

//            next_x_vals = xy[0];
//            next_y_vals = xy[1];


            // Spline Smoothing XY line
            tk::spline splX;
            splX.set_points(TT, xy[0]);

            tk::spline splY;
            splY.set_points(TT, xy[1]);

            vector<double> XX_smooth;
            vector<double> YY_smooth;
            t = 0.0;
            double ts = PATH_TIMESTEP;
            while (t <= T) {
              XX_smooth.push_back(splX(t));
              YY_smooth.push_back(splY(t));
              t += ts;
            }

            next_x_vals = XX_smooth;
            next_y_vals = YY_smooth;
            */






//            for (int i = 0; i < SS.size(); ++i) {
//              vector<double> xy = getXY(SS[i], DD[i], map_waypoints_s, map_waypoints_x, map_waypoints_y);
//              next_x_vals.push_back(xy[0]);
//              next_y_vals.push_back(xy[1]);
//              cout << i << ": " << xy[0] << ", " << xy[1] << endl;
//            }
            hState.path = true;
          } else {
            // copy previous path as a whole
            for (int i = 0; i < previous_path_x.size(); ++i) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            print_vals(next_x_vals, next_y_vals);


          }

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
    testLocal(TEST_JSON_1, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  } else {
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
