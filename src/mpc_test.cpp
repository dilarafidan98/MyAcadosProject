#include <iostream>
#include <cstdlib> // For exit() function
//standart
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "c_generated_code/acados_solver_quadcopter.h"


// blasf
#include "blasfeo/include/blasfeo_d_aux_ext_dep.h"


#define NX     QUADCOPTER_NX
#define NU     QUADCOPTER_NU
#define NBX0   QUADCOPTER_NBX0
using namespace std;


int main() {
   // Create the ACADOS capsule
   quadcopter_solver_capsule *acados_ocp_capsule = quadcopter_acados_create_capsule();


   // Check for creation failure
   if (acados_ocp_capsule == nullptr) {
       cerr << "Failed to create acados_ocp_capsule. Exiting.\n";
       exit(1);
   }


   // Change the number of shooting intervals in C without new code generation
   int N = QUADCOPTER_N;
   double* new_time_steps = nullptr;
   int status = quadcopter_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);


   if (status) {
       cerr << "quadcopter_acados_create() returned status " << status << ". Exiting.\n";
       exit(1);
   }


   // Other configurations...
   ocp_nlp_config *nlp_config = quadcopter_acados_get_nlp_config(acados_ocp_capsule);
   ocp_nlp_dims *nlp_dims = quadcopter_acados_get_nlp_dims(acados_ocp_capsule);
   ocp_nlp_in *nlp_in = quadcopter_acados_get_nlp_in(acados_ocp_capsule);
   ocp_nlp_out *nlp_out = quadcopter_acados_get_nlp_out(acados_ocp_capsule);
   ocp_nlp_solver *nlp_solver = quadcopter_acados_get_nlp_solver(acados_ocp_capsule);
   void *nlp_opts = quadcopter_acados_get_nlp_opts(acados_ocp_capsule);
  




   double lbx0[NBX0], ubx0[NBX0];
   double lbx0[NBX0];
   double ubx0[NBX0];
   lbx0[0] = 3.490658503988659;
   ubx0[0] = 3.490658503988659;
   lbx0[1] = 0;
   ubx0[1] = 0;
   lbx0[2] = 0;
   ubx0[2] = 0;
   lbx0[3] = 4.71238898038469;
   ubx0[3] = 4.71238898038469;
   lbx0[4] = 0;
   ubx0[4] = 0;
   lbx0[5] = 0;
   ubx0[5] = 0;


   ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
   ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);


   // Initialization for state values and control input
   double x_init[NX] = {0};
   double u0[NU] = {0};


   // Prepare evaluation
   int NTIMINGS = 1;
   double min_time = 1e12;
   double kkt_norm_inf;
   double elapsed_time;
   int sqp_iter;
   // ... (Other variables)


   double xtraj[NX * (N+1)];
   double utraj[NU * N];


   // Solve ocp in loop
   // ... (Loop implementation)
   int rti_phase = 0;


   for (int ii = 0; ii < NTIMINGS; ii++)
   {
       // initialize solution
       for (int i = 0; i < N; i++)
       {
           ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x_init);
           ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
       }
       ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x_init);
       ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "rti_phase", &rti_phase);
       status = quadcopter_acados_solve(acados_ocp_capsule);
       ocp_nlp_get(nlp_config, nlp_solver, "time_tot", &elapsed_time);
       min_time = MIN(elapsed_time, min_time);
   }
  




    /* print solution and statistics */
   for (int ii = 0; ii <= nlp_dims->N; ii++)
       ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
       d_print_exp_tran_mat( NX, N+1, xtraj, NX);
   for (int ii = 0; ii < nlp_dims->N; ii++)
       ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);
       d_print_exp_tran_mat( NX, N+1, xtraj, NX);


   // Print solution and statistics
   cout << "\n--- xtraj ---\n";
   // ... (Call to print function for xtraj)
   cout << "\n--- utraj ---\n";
   // ... (Call to print function for utraj)


   // Final status check and print
   if (status == ACADOS_SUCCESS) {
       cout << "quadcopter_acados_solve(): SUCCESS!\n";
   } else {
       cerr << "quadcopter_acados_solve() failed with status " << status << ".\n";
   }


   // get solution
   ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
   ocp_nlp_get(nlp_config, nlp_solver, "sqp_iter", &sqp_iter);


   quadcopter_acados_print_stats(acados_ocp_capsule);


   printf("\nSolver info:\n");
   printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
          sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);


   // Free solver
   status = quadcopter_acados_free(acados_ocp_capsule);
   if (status) {
       cerr << "quadcopter_acados_free() returned status " << status << ".\n";
   }


   // Free solver capsule
   status = quadcopter_acados_free_capsule(acados_ocp_capsule);
   if (status) {
       cerr << "quadcopter_acados_free_capsule() returned status " << status << ".\n";
   }


   return status;
}
