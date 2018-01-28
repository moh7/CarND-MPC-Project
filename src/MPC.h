#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

#define NUMBER_OF_STEPS 10
#define DT 0.1

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
#define LF 2.67

// reference speed
#define V_REF 70

// Set weights parameters for the cost function
/*
#define W_CTE 8.4
#define W_EPSI 0.32
#define W_V 0.261
#define W_DELTA 600000
#define W_A 17.1
#define W_DDELTA 0.01
#define W_DA 0.00001
*/
#define W_CTE 2
#define W_EPSI 10
#define W_V 1
#define W_DELTA 1000
#define W_A 1
#define W_DDELTA 0.5
#define W_DA 0.5

using namespace std;

class MPC {

 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  vector<double> mpc_x;
  vector<double> mpc_y;
};

#endif /* MPC_H */
