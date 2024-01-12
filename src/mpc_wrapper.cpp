//This file is C++ wrapper to use specific functions
#include "../include/mpc_test.h"


namespace mpc_solvers
{
namespace quadcopter_solvers
{
/**
* @brief Internal function to set initial guesses for the solver output variable
*       This function just wraps the ocp_nlp_out_set call
*
* @param stage current stage of the mpc (0,....,horizon-1)
* @param type type either "x" or "u"
* @param constraint constraints must have same dimension as x or u (depending on type)
*/
void AcadosQuadcopterSolver::setOutputInitialGuess(int stage, std::string type, double constraint[])
{
 ocp_nlp_out_set(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), stage, type.c_str(), constraint);
}
/**
* @brief Sets an input bound constraint.
*  This function just wraps the ocp_nlp_constraints_model_set call
*
* @param stage current stage of the mpc (0,....,horizon-1)
* @param type type either "x" or "u"
* @param constraint constraints must have same dimension as x or u (depending on type)
*/
void AcadosQuadcopterSolver::setInputBoundConstraint(int stage, std::string type, double constraint[])
{
 ocp_nlp_constraints_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), stage, type.c_str(), constraint);
}
/**
* @brief Get the Last Solution and stores it in x and u
*
* @param x states, size state dimension x horizon length
* @param u input, size input dimension x horizon length
*/


void AcadosQuadcopterSolver::getLastSolution(double x[], double u[])
{
 for (int n = 0; n < getHorizonLength(); n++)
 {
   ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "x", &x[n * getStateDimension()]);  // NOLINT
   ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "u", &u[n * getInputDimension()]);  // NOLINT
 }
}


/**
* @brief Construct a new Acados Pacejka Mpcc Solver:: Acados Pacejka Mpcc Solver object
*
*/
AcadosQuadcoptercSolver::AcadosQuadcoptersetInputBoundConstraintSolver()
{
 // initialize acados solver
 acados_ocp_capsule_.reset(quadcopter_acados_create_capsule());
 quadcopter_acados_create(acados_ocp_capsule_.get());


 nlp_config_.reset(quadcopter_acados_get_nlp_config(acados_ocp_capsule_.get()));
 nlp_dims_.reset(quadcopter_acados_get_nlp_dims(acados_ocp_capsule_.get()));
 nlp_in_.reset(quadcopter_acados_get_nlp_in(acados_ocp_capsule_.get()));
 nlp_out_.reset(quadcopter_acados_get_nlp_out(acados_ocp_capsule_.get()));
 nlp_solver_.reset(quadcopter_acados_get_nlp_solver(acados_ocp_capsule_.get()));
}


/**
* @brief Get the Horizon Length
*
* @return const int
*/
const int AcadosQuadcopterSolver::getHorizonLength() const
{
 return nlp_dims_->N;
}


/**
* @brief Set the Initial State Constraint. The provided array must have the same length as the state dimension
*
* @param constraint
*/
void AcadosQuadcopterSolver::setInitialState(double constraint[])
{
 setInputBoundConstraint(0, "lbx", constraint);
 setInputBoundConstraint(0, "ubx", constraint);
}
/**
* @brief Sets an initial guess for the state at stage "stage" of the solver.
* The provided array must have the same length as the state dimension
*
* @param constraint
*/
void AcadosQuadcopterSolver::setStateInitialGuess(int stage, double constraint[])
{
 setOutputInitialGuess(stage, "x", constraint);
}
/**
* @brief Sets an initial guess for the input at stage "stage" of the solver.
* The provided array must have the same length as the input dimension
*
* @param constraint
*/
void AcadosQuadcopterSolver::setInputInitialGuess(int stage, double constraint[])
{
 setOutputInitialGuess(stage, "u", constraint);
}






//--------------------------Just in case implement update parameter function---------------------------------------
void AcadosQuadcopterSolver::updateParams(int stage, double *value, int np_ )
{
   quadcopter_acados_update_params(acados_ocp_capsule_.get(),stage,mpc_parameters,np_)


}
//----------------------------------------------------------------------------------------------------------------------


/**
* @brief Solves the optimization problems and stores the solution in x and u.
*
* @param x State array or point with size N*StateDimenstion
* @param u Input array or point with size N*Inputdimension
* @return const int, return code. If no error occurred, return code is zero
*/
int AcadosQuadcopterSolver::solve(double x[], double u[])
{
 int status = quadcopter_acados_solve(acados_ocp_capsule_.get());
 if (status)
 {
   return status;
 }
 getLastSolution(x, u);
 return status;
}


double AcadosQuadcopterSolver::getSamplePeriod()
{
 return *(nlp_in_->Ts);
}
















}; // namespace quadcopter_solvers






}//namespace mpc_solvers
