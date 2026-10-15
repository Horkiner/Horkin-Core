# Simulation backend. Implements ports; may use kine/dyn for plant.
# Must NOT link hal.

add_library(horkin_sim STATIC
  plant.cpp
  joint_sim.cpp
  servo_sim.cpp
)
target_link_libraries(horkin_sim
  PUBLIC horkin_ports horkin_dyn
  PRIVATE ${HORKIN_PINOCCHIO}
)
target_include_directories(horkin_sim PUBLIC "${HORKIN_INCLUDE_ROOT}")

if(HORKIN_BUILD_TESTS)
  add_executable(test_plant test/test_plant.cpp)
  target_link_libraries(test_plant PRIVATE horkin_sim)
  target_compile_definitions(test_plant PRIVATE
    HORKIN_TEST_URDF="${CMAKE_SOURCE_DIR}/config/urdf/v6_mc.urdf")
  
  add_executable(test_joint_sim test/test_joint_sim.cpp)
  target_link_libraries(test_joint_sim PRIVATE horkin_sim)
  target_compile_definitions(test_joint_sim PRIVATE
    HORKIN_TEST_URDF="${CMAKE_SOURCE_DIR}/config/urdf/v6_mc.urdf")

  add_executable(test_servo_sim test/test_servo_sim.cpp)
  target_link_libraries(test_servo_sim PRIVATE horkin_sim)
endif()
