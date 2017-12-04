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

  // MPC is initialized here!
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

          vector<double> ptsx = j[1]["ptsx"]; // x waypoints, global coordinates
          vector<double> ptsy = j[1]["ptsy"]; // y waypoints, global coordinates
          double px = j[1]["x"];  // x position, vehicle coordinates
          double py = j[1]["y"];  // y position, vehicle coordinates
          double speed_MPH = j[1]["speed"];  // CAUTION: in MPH!!
          double psi = j[1]["psi"]; // heading, global coordinates
          double SWA = j[1]["steering_angle"];  // CAUTION: CCW positive!!
          double throttle = j[1]["throttle"]; // [-1, 1]

          //transform waypoints from global to vehicle coordinate system
          Eigen::VectorXd ptx(ptsx.size()), pty(ptsy.size()); // desired path waypoints, vehicle coordinates
          for (int i = 0; i < ptx.size(); i++) {
            ptx[i] = ptsx[i]-px;
            pty[i] = ptsy[i]-py;
          }

          std::cout << "x:" << ptx << std::endl;
          std::cout << "y:" << pty << std::endl;

          // fit a polynomial to the above x and y coordinates
          int poly_order = 3;
          auto coeffs = polyfit(ptx, pty, poly_order);
          std::cout << "coeffs:" << coeffs << std::endl;

          // build the desired path
          vector<double> next_x, next_y; // desired path, vehicle coordinates
          int n_points = 10;
          int xstepsize = 2;
          for (int i = 0; i < n_points; i++) {
            double xstep = (double) i*xstepsize;
            next_x.push_back(xstep);
            next_y.push_back(polyeval(coeffs, xstep));
          }

          double cte = pty[0]; // cross-track error, m
          double epsi = psi; // heading error, radians

		      Eigen::VectorXd state(6); // The states are: x, y, psi, v, cte, epsi
		      state << 0., 0., 0., MPH2mps(speed_MPH), cte, epsi;

		      //auto vars = mpc.Solve(state, coeffs);

          double steer_value=0.; // = vars[3];
          double throttle_value=0.; // = vars[4];

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
#if (0)
          msgJson["mpc_x"] = mpc_x;
          msgJson["mpc_y"] = mpc_y;
#endif
          msgJson["next_x"] = next_x;
          msgJson["next_y"] = next_y;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
