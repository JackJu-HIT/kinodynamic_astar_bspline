find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)

include_directories(
  include
  localmap_esdf/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(fast_sdf_map STATIC
  localmap_esdf/src/sdf_map.cpp
)

add_library(fast_trajopt STATIC
  src/bspline_replanner.cpp
  src/grad_band_optimizer.cpp
  src/non_uniform_bspline.cpp
  src/bezier_base.cpp
)

target_link_libraries(fast_trajopt
  fast_sdf_map
  ${NLOPT_LIBRARIES}
)
