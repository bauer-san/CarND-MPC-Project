#define DEBUG 0
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

/*
In this project you'll implement Model Predictive Control to drive the car around the track. 
This time however you're not given the cross track error, you'll have to calculate that yourself! 

Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.
*/

namespace {
using CppAD::AD;

const size_t N=10;
const double dt=0.1;

// NOTE: DON'T CHANGE THIS IT WAS CAREFULLY CHOSEN!!!
const double Lf = 2.67;

const double ref_v = 25.;	// reference longitudinal velocity, mps
const double ref_cte = 0.; // reference cte, meters
const double ref_epsi = 0.*M_PI / 180.; // reference psi, radians

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t deltaLBO_start = epsi_start + N;
size_t delta_start = deltaLBO_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  Eigen::VectorXd coeffs;
  // Coefficients of the fitted polynomial.
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // fg a vector of constraints, vars is a vector of constraints.

    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += 2*CppAD::pow(vars[cte_start + t] - ref_cte, 2);
#if(DEBUG)
      std::cout << "cte: " << fg[0] << std::endl;
#endif
      fg[0] += CppAD::pow(vars[epsi_start + t] - ref_epsi, 2);
#if(DEBUG)
      std::cout << "epsi: " << fg[0] << std::endl;
#endif
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
#if(DEBUG)
      std::cout << "v: " << fg[0] << std::endl;
#endif
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 20*CppAD::pow(vars[deltaLBO_start + t + 1] - vars[deltaLBO_start + t], 2);
      fg[0] += 20*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    fg[1 + deltaLBO_start] = vars[deltaLBO_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
      AD<double> deltaLBO1 = vars[deltaLBO_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      AD<double> deltaLBO0 = vars[deltaLBO_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // desired path and
      AD<double> f0 =   coeffs[0]
                      + coeffs[1] * x0
                      + coeffs[2] * x0 * x0
                      + coeffs[3] * x0 * x0 * x0;
      AD<double> psides0 = CppAD::atan(      coeffs(1)
                                       + 2 * coeffs(2) * x0
                                       + 3 * coeffs(3) * x0 * x0);

      // Recall the equations for the kinematic motion model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      // deltaLBO[t+1] = delta[t]
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * deltaLBO0 / Lf * dt); // corrected for steering orientation
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((y0 - f0) + (v0 * CppAD::sin(epsi0) * dt)); // original: cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) - v0 * deltaLBO0 / Lf * dt); // corrected for steering orientation
      fg[1 + deltaLBO_start + t] = deltaLBO1 - (0.*deltaLBO0 + 1.0*delta0); // deltaLBO[t+1] = filtered(delta[t])
    }
  }
};
}

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

tuple<vector<double>, vector<double>, double> MPC::Solve(Eigen::VectorXd x0, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  const double x = x0[0];
  const double y = x0[1];
  const double psi = x0[2];
  const double v = x0[3];
  const double cte = x0[4];
  const double epsi = x0[5];
  const double delta = x0[6];

  const int n_states = 7;

  // Number of variables (includes both states and inputs)
  size_t n_vars = N * n_states + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * n_states;

  // Initial value of the independent variables.
  // Should be 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  vars[deltaLBO_start] = delta;

  // Lower and upper limits for states, inputs
  Dvector vars_lowerbound(n_vars), vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < deltaLBO_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  double v_comp = std::max(0.01, v / 8.);
  for (int i = deltaLBO_start; i < delta_start; i++) {  //DELTA LBO
    vars_lowerbound[i] = -0.436332 / v_comp;
    vars_upperbound[i] = 0.436332 / v_comp;
  }
  for (int i = delta_start; i < a_start; i++) { // delta
    vars_lowerbound[i] = -0.436332 / v_comp;
    vars_upperbound[i] = 0.436332 / v_comp;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -0.6;
    vars_upperbound[i] = 0.6;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints), constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  constraints_lowerbound[deltaLBO_start] = delta;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;
  constraints_upperbound[deltaLBO_start] = delta;

  //
  // NOTE: Most of this stuff you don't have to worry about
  //

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound, constraints_upperbound, fg_eval,
                                        solution);
  //
  // Check some of the solution values
  //
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  vector<double> x1; // optimized position x,y
  for (int i = 0; i < N; i++) { // ravel the x and y coordinates
    x1.push_back(solution.x[x_start+i]);  // x
    x1.push_back(solution.x[y_start+i]);  // y
  }

#if(DEBUG)
  std::cout << "solution.x" << std::endl;
  for (int i = deltaLBO_start; i < delta_start; i++) {
    std::cout << i << ", " << solution.x[i] << ", " << solution.x[i+N] << std::endl;
  }
#endif

  auto u1 = { solution.x[delta_start] / (25.*M_PI / 180.), solution.x[a_start] };
  
  auto cost = solution.obj_value;
  
  return std::make_tuple(u1, x1, cost);
}