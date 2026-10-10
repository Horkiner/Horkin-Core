# 刚体模型：持有 URDF / Pinocchio。运动学算法在 kine/，动力学算法在 dyn/。

add_library(horkin_model STATIC
  robot_model.cpp
)
target_link_libraries(horkin_model
  PUBLIC horkin_types
  PRIVATE ${HORKIN_PINOCCHIO}
)
target_include_directories(horkin_model PUBLIC "${HORKIN_INCLUDE_ROOT}")

target_precompile_headers(horkin_model PRIVATE
  <pinocchio/fwd.hpp>
  <pinocchio/parsers/urdf.hpp>
)
