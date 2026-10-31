#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <memory>
#include <utility>
#include "plan_env/grid_map.h"

namespace fast_planner {
using std::shared_ptr;
using std::vector;

// Adapter connecting the 2D kinodynamic search to the local occupancy map.
class EDTEnvironment {
 public:
  using Ptr = std::shared_ptr<EDTEnvironment>;
  explicit EDTEnvironment(const std::shared_ptr<GridMap2D>& map)
      : map_(map), sdf_map_(std::make_shared<Occupancy>(map)) {}
  struct Occupancy {
    explicit Occupancy(std::shared_ptr<GridMap2D> map) : map_(std::move(map)) {}
    void getRegion(Eigen::Vector3d& origin, Eigen::Vector3d& size) const {
      origin.head<2>() = map_->origin();
      origin.z() = -0.5;
      size.head<2>() = map_->mapSize().cast<double>() * map_->resolution();
      size.z() = 1.0;
    }
    int getInflateOccupancy(const Eigen::Vector3d& p) const {
      return (std::abs(p.z()) > 1e-3 ||
              map_->queryInflated(p.head<2>()) != GridMap2D::CellState::kFree) ? 1 : 0;
    }
    std::shared_ptr<GridMap2D> map_;
  };
 private:
  std::shared_ptr<GridMap2D> map_;
 public:
  std::shared_ptr<Occupancy> sdf_map_;
};
}
