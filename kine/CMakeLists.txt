# 运动学算法。模型对象在 horkin_model。

add_library(horkin_kine STATIC
  fk.cpp
  jacobian.cpp
  ik_dls.cpp
)
target_link_libraries(horkin_kine
  PUBLIC horkin_model
  PRIVATE ${HORKIN_PINOCCHIO}
)
target_include_directories(horkin_kine PUBLIC "${HORKIN_INCLUDE_ROOT}")

target_precompile_headers(horkin_kine PRIVATE
  <pinocchio/fwd.hpp>
  <pinocchio/algorithm/kinematics.hpp>
  <pinocchio/algorithm/jacobian.hpp>
)

if(HORKIN_BUILD_TESTS)
  add_executable(test_fk test/test_fk.cpp)
  target_link_libraries(test_fk PRIVATE horkin_kine)
  target_compile_definitions(test_fk PRIVATE
    HORKIN_TEST_URDF="${CMAKE_SOURCE_DIR}/config/urdf/v6_mc.urdf")

  add_executable(test_ik test/test_ik.cpp)
  target_link_libraries(test_ik PRIVATE horkin_kine)
  target_compile_definitions(test_ik PRIVATE
    HORKIN_TEST_URDF="${CMAKE_SOURCE_DIR}/config/urdf/v6_mc.urdf")
endif()
