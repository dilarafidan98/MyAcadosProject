cmake_minimum_required(VERSION 3.10)
project(TestAcadosMPC)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# Path to the ACADOS installation
set(ACADOS_SOURCE_DIR "/home/dilara/acados")

include_directories(
  include
  src/c_generated_code
  $ENV{ACADOS_SOURCE_DIR}/include
  $ENV{ACADOS_SOURCE_DIR}/include/acados
  $ENV{ACADOS_SOURCE_DIR}/include/blasfeo/include
  $ENV{ACADOS_SOURCE_DIR}/include/hpipm/include
)

find_library(ACADOS_LIB 
  NAMES acados
        hpipm
        blasfeo
  PATHS $ENV{ACADOS_SOURCE_DIR}/lib
)

file(GLOB_RECURSE GENERATED_C_CODE RELATIVE ${CMAKE_SOURCE_DIR} "src/c_generated_code/*.c")
# Remove matlab dependency
list(FILTER GENERATED_C_CODE EXCLUDE REGEX ".*sfunction.*\\.c$")
list(FILTER GENERATED_C_CODE EXCLUDE REGEX ".*sim.*\\.c$")

add_library(${PROJECT_NAME}
    src/mpc_test.cpp
    src/mpc_wrapper.cpp
    src/mpc_rate_control.cpp
     # Main solver file used to interface with MPC controller
    
    ${GENERATED_C_CODE} # Generated c code
)

# build as an executable
# add_executable(${PROJECT_NAME}
#     src/mpc_test.cpp # Main solver file used to interface with MPC controller
#     ${GENERATED_C_CODE} # Generated c code
# )

# Link the libraries
target_link_libraries(${PROJECT_NAME} ${ACADOS_LIB}
)


# install()
# export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include LIBRARIES ${PROJECT_NAME} ${ACADOS_LIB})


