# 力控。单轴导纳先落地；6D / 选择矩阵 / FSM 后加。

add_library(horkin_control_force STATIC
    admittance_axis.cpp
)
target_link_libraries(horkin_control_force PUBLIC horkin_types)
target_include_directories(horkin_control_force PUBLIC "${HORKIN_INCLUDE_ROOT}")

if(HORKIN_BUILD_TESTS)
  add_executable(test_admittance_axis test/test_admittance_axis.cpp)
  target_link_libraries(test_admittance_axis PRIVATE horkin_control_force)
endif()

# Example when implementing:
# add_library(horkin_control_force STATIC
#   admittance_axis.cpp
#   admittance_6d.cpp
#   selection.cpp
#   contact_fsm.cpp
#   safety_limit.cpp
# )
# target_link_libraries(horkin_control_force PUBLIC horkin_types)
# # Prefer calling kine/dyn as free functions; link when needed:
# # target_link_libraries(horkin_control_force PUBLIC horkin_kine horkin_dyn)
# target_include_directories(horkin_control_force PUBLIC "${HORKIN_INCLUDE_ROOT}")
