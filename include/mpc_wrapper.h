
//This file is C++ wrapper header file to use specific functions
#ifndef MPC_WRAPPER_MPC_WRAPPER
#define MPC_WRAPPER_MPC_WRAPPER


#include <acados/utils/math.h>
#include <acados/utils/print.h>
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "../src/c_generated_code/acados_solver_quadcopter.h"


#include <memory>


namespace mpc_solvers
{
namespace quadcopter_solvers
{


class AcadosQuadcopterSolver : public QuadcopterSolver
{
private:
 std::unique_ptr<nlp_solver_capsule> acados_ocp_capsule_;
 std::unique_ptr<ocp_nlp_config> nlp_config_;
 std::unique_ptr<ocp_nlp_dims> nlp_dims_;
 std::unique_ptr<ocp_nlp_in> nlp_in_;
 std::unique_ptr<ocp_nlp_out> nlp_out_;
 std::unique_ptr<ocp_nlp_solver> nlp_solver_;




 const static int np_=0;
 double mpc_parameters[np_];






 /**
  * @brief Internal function to set initial guesses for the solver output variable
  *       This function just wraps the ocp_nlp_out_set call
  *
  * @param stage current stage of the mpc (0,....,horizon-1)
  * @param type type either "x" or "u"
  * @param constraint constraints must have same dimension as x or u (depending on type)
  */
 void setOutputInitialGuess(int stage, std::string type, double constraint[]);


 /**
  * @brief Sets an input bound constraint.
  *  This function just wraps the ocp_nlp_constraints_model_set call
  *
  * @param stage current stage of the mpc (0,....,horizon-1)
  * @param type type either "x" or "u"
  * @param constraint constraints must have same dimension as x or u (depending on type)
  */
 void setInputBoundConstraint(int stage, std::string type, double constraint[]);
 /**
  * @brief Get the Last Solution and stores it in x and u
  *
  * @param x states, size state dimension x horizon length
  * @param u input, size state dimension x horizon length
  */
 void getLastSolution(double x[], double u[]);


public:
 AcadosQuadcopterSolver();


 /**
  * @brief Get the Horizon Length
  *
  * @return const int
  */
 const int getHorizonLength() const override;


 /**
  * @brief Set the Initial State Constraint. The provided array must have the same length as the state dimension
  *
  * @param constraint
  */
 void setInitialState(double constraint[]) override;
 /**
  * @brief Sets an initial guess for the state at stage "stage" of the solver.
  * The provided array must have the same length as the state dimension
  *
  * @param constraint
  */
 void setStateInitialGuess(int stage, double constraint[]) override;
 /**
  * @brief Sets an initial guess for the input at stage "stage" of the solver.
  * The provided array must have the same length as the input dimension
  *
  * @param constraint
  */
 void setInputInitialGuess(int stage, double constraint[]) override;




 void updateParams(int stage, double *value, int np_ ) override;


 /**
  * @brief Solves the optimization problems and stores the solution in x and u.
  *
  * @param x State array or point with size N*StateDimenstion
  * @param u Input array or point with size N*Inputdimension
  * @return const int, return code. If no error occurred, return code is zero
  */
 int solve(double x[], double u[]) override;


 /**
  * @brief Returns the sample period in seconds
  *
  * @return double
  */
 double getSamplePeriod() override;
};


}  // namespace quadcopter_solvers
}  // namespace mpc_solvers
#endif /* INCLUDE_MPC_WRAPPER \