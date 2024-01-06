cmake_minimum_required(VERSION 3.10)
project(MyAcadosProject)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# Path to the ACADOS installation
set(ACADOS_PATH /home/dilara/acados)

# Include directories for ACADOS, BLASFEO, and HPIPM
include_directories(${ACADOS_PATH}/include)
include_directories(${ACADOS_PATH}/external/blasfeo/include)
include_directories(${ACADOS_PATH}/external/hpipm/include)
include_directories(${PROJECT_SOURCE_DIR}/include)
# Link directories for ACADOS, BLASFEO, and HPIPM
link_directories(${ACADOS_PATH}/lib)
link_directories(${ACADOS_PATH}/external/blasfeo/lib)
link_directories(${ACADOS_PATH}/external/hpipm/lib)

add_executable(${PROJECT_NAME}
        MyMPCTest.cpp # Main solver file used to interface with MPC controller
    )


# Link libraries from the lib/ directory
target_link_libraries(${PROJECT_NAME} lib/acados_sim_solver_quadcopter.o,
                                      lib/acados_solver_quadcopter.o,
                                      lib/libacados_ocp_solver_quadcopter.so,
                                      lib/libacados_sim_solver_quadcopter.so,
                                      lib/libacados_solver_quadcopter.so,
                                      lib/main_quadcopter.o,
                                      lib/main_sim_quadcopter.o,
                                      lib/quadcopter_model/quadcopter_impl_dae_fun_jac_x_xdot_z.o,
                                      lib/quadcopter_model/quadcopter_impl_dae_fun.o,
                                      lib/quadcopter_model/quadcopter_impl_dae_jac_x_xdot_u_z.o,
                                      lib/quadcopter_cost/quadcopter_cost_y_0_fun_jac_ut_xt.o,
                                      lib/quadcopter_cost/quadcopter_cost_y_0_fun.o,
                                      lib/quadcopter_cost/quadcopter_cost_y_0_hess.o,
                                      lib/quadcopter_cost/quadcopter_cost_y_e_fun_jac_ut_xt.o,
                                      lib/quadcopter_cost/quadcopter_cost_y_e_fun.o,
                                      lib/quadcopter_cost/quadcopter_cost_y_e_hess.o,
                                      lib/quadcopter_cost/quadcopter_cost_y_fun_jac_ut_xt.o,
                                      lib/quadcopter_cost/quadcopter_cost_y_fun.o,
                                      lib/quadcopter_cost/quadcopter_cost_y_hess.o,
)


# Link ACADOS, BLASFEO, and HPIPM to your project
target_link_libraries(MyAcadosProject acados hpipm blasfeo)