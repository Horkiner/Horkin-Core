# 动力学算法。模型对象在 horkin_model。
add_library(horkin_dyn STATIC
  rnea.cpp
  gravity.cpp
  mass.cpp
)
target_link_libraries(horkin_dyn
  PUBLIC horkin_model
  PRIVATE ${HORKIN_PINOCCHIO}
)
target_include_directories(horkin_dyn PUBLIC "${HORKIN_INCLUDE_ROOT}")

if(HORKIN_BUILD_TESTS)
  add_executable(test_rnea test/test_rnea.cpp)
  target_link_libraries(test_rnea PRIVATE horkin_dyn)
  target_compile_definitions(test_rnea PRIVATE
    HORKIN_TEST_URDF="${CMAKE_SOURCE_DIR}/config/urdf/v6_mc.urdf")

  add_executable(test_mass test/test_mass.cpp)
  target_link_libraries(test_mass PRIVATE horkin_dyn)
  target_compile_definitions(test_mass PRIVATE
    HORKIN_TEST_URDF="${CMAKE_SOURCE_DIR}/config/urdf/v6_mc.urdf")
endif()
