#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

//#define PRINT_VARS
//#define PRINT_OPERATOR
//#define PRINT_F_EVAL
//#define PRINT_STATE
//#define INTERP_SECOND
#define PRINT_OUTPUT

using namespace std;

class MPC {
 public:
 
  MPC();
  
  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
