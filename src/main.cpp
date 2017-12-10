#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "MPC.h"
#include "json.hpp"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double MPH2mps(double x) { return x * 0.44704; }

//
// Helper functions to fit and evaluate polynomials.
// These functions are from UDACITY mpc_to_line quiz
//

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

int main() {
  uWS::Hub h;

  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data));
      
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
     
          // j[1] is the data JSON object:
          //    {"ptsx":[-32.16173, -43.49173, -61.09, -78.29172, -93.05002, -107.7717], 
          //    "ptsy" : [113.361, 105.941, 92.88499, 78.73102, 65.34102, 50.57938], 
          //    "psi_unity" : 4.120315, 
          //    "psi" : 3.733667, 
          //    "x" : -40.62008, 
          //    "y" : 108.7301, 
          //    "steering_angle" : 0, 
          //    "throttle" : 0, 
          //    "speed" : 6.619145E-06}

          vector<double> ptsx = j[1]["ptsx"];   // x waypoints, global coordinates
          vector<double> ptsy = j[1]["ptsy"];   // y waypoints, global coordinates
          double px = j[1]["x"];                // x position, vehicle coordinates
          double py = j[1]["y"];                // y position, vehicle coordinates
          double speedMPH = j[1]["speed"];      // velocity, MPH
          double psi = j[1]["psi"];             // heading, DEGREES? global coordinates
          double SWA = j[1]["steering_angle"];  // radians CAUTION: CCW positive!!
          double throttle = j[1]["throttle"];   // [-1, 1]

          /*
              Transform waypoints of DESIRED PATH from 
              GLOBAL to VEHICLE coordinate system
          */
          Eigen::VectorXd ptx(ptsx.size()), pty(ptsy.size()); // desired path waypoints, vehicle coordinates
          for (int i = 0; i < ptx.size(); i++) {
            double tempx = ptsx[i] - px;
            double tempy = ptsy[i] - py;
            //double psi_rad = deg2rad(psi);  //TODO: strange
            double psi_rad = psi;
            ptx[i] = tempx*cos(0 - psi_rad) - tempy*sin(0 - psi_rad);
            pty[i] = tempx*sin(0 - psi_rad) + tempy*cos(0 - psi_rad);
          }

          /*
              Fit a polynomial to the above x and y coordinates
          */
          int poly_order = 3;
          auto coeffs = polyfit(ptx, pty, poly_order);

          double cte = coeffs[0];               // cross-track error, m  (SIMPLIFIED from 'polyeval(coeffs, 0.)')
          double epsi = -atan(coeffs[1]);       // heading error, radians

		      Eigen::VectorXd ego_vehicle_state(7); // The states are: x=0, y=0, psi=0, v, cte, epsi, SWA
		      ego_vehicle_state << 0., 0., 0., MPH2mps(speedMPH), cte, epsi, SWA;
		      auto vars = mpc.Solve(ego_vehicle_state, coeffs);

          json msgJson;

          /*
              Actuator commands
          */
          auto u1 = std::get<0>(vars); // first element returned from solution has the actuator command: swa, throttle
          double cmdSWA = u1[0];
          double cmdTHROTTLE = u1[1];
          msgJson["steering_angle"] = cmdSWA;
          msgJson["throttle"] = cmdTHROTTLE;

          /*
              Unravel the path from the MPC solution

              The MPC path is visualized in the simulator as a GREEN line
          */
          auto x1 = std::get<1>(vars); // second element returned from solution has the x and y coordinates in a flattened vector
          vector<double> mpc_x, mpc_y; // solver path, VEHICLE coordinates
          for (int i = 0; i < 25; i++) { //unravel the x and y coordinates returned from solver // TODO: fix this magic number
            mpc_x.push_back(x1[2*i]);
            mpc_y.push_back(x1[2*i+1]);
          }
          msgJson["mpc_x"] = mpc_x;
          msgJson["mpc_y"] = mpc_y;

          /*
              Build the DESIRED path

              The DESIRED path is visualized in the simulator as a YELLOW line
          */
          vector<double> next_x, next_y; // desired path, VEHICLE coordinates
          int n_points = 10;
          int xstepsize = 2;
          for (int i = 0; i < n_points; i++) {
            double xstep = (double)std::pow(1.5, i)*xstepsize;
            next_x.push_back(xstep);
            next_y.push_back(polyeval(coeffs, xstep));
          }
          msgJson["next_x"] = next_x;
          msgJson["next_y"] = next_y;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
#if(1)
          std::cout << px << ", "<< py << ", " << psi << ", " << speedMPH << ", " << cte << ", " << epsi << ", " << SWA << ", " << cmdSWA << ", " << cmdTHROTTLE << std::endl;
#endif
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

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
