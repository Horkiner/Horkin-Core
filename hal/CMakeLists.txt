add_library(horkin_hal STATIC
  motor_dm.cpp
  joint_trans.cpp
)
target_link_libraries(horkin_hal PUBLIC horkin_ports)
target_include_directories(horkin_hal PUBLIC "${HORKIN_INCLUDE_ROOT}")
if(HORKIN_BUILD_TESTS)
  add_executable(test_motor_dm test/test_motor_dm.cpp)
  target_link_libraries(test_motor_dm PRIVATE horkin_hal)
  add_executable(test_joint_trans test/test_joint_trans.cpp)
  target_link_libraries(test_joint_trans PRIVATE horkin_hal)
endif()
