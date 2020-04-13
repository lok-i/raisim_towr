cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project(raisim_towr)
find_package(ifopt)
find_package(raisimOgre CONFIG REQUIRED)
find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

include_directories (${EIGEN3_INCLUDE_DIRS})
find_package(towr 1.2 REQUIRED)


function(create_executable app_name file_name)
  add_executable(${app_name} ${file_name})

  target_link_libraries(${app_name} PUBLIC raisim::raisimOgre)
  target_link_libraries(${app_name} PUBLIC ifopt::ifopt_ipopt) 
  target_link_libraries(${app_name} PUBLIC towr::towr) # adds include directories and libraries

  
  target_include_directories(${app_name} PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIRS})
  
  target_compile_definitions(${app_name} PRIVATE "-DEXAMPLE_ROBOT_RESOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/rsc/")
endfunction()




#add_executable(towr_monoped towr_monoped.cpp) # Your custom variables, costs and constraints added to TOWR

#target_link_libraries(towr_monoped PUBLIC ifopt::ifopt_ipopt) 
#target_link_libraries(towr_monoped PUBLIC towr::towr) # adds include directories and libraries

#create_executable(raisim_ik monoped_raisim_ik.cpp)
create_executable(towr_raisim monoped_towr_raisim_ik_2.cpp)
create_executable(ik_test ik_test.cpp)

