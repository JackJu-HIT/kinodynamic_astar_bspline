#include "planner_interface.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace kinodynamic_astar_ros {

void PlannerInterface::initParam(double max_vel, double max_acc) {
  max_vel_ = max_vel;
  max_acc_ = max_acc;
}

void PlannerInterface::initMap(double x_size, double y_size, double resolution,
                               double inflate_values) {
  resolution_ = resolution;
  const Eigen::Vector2i map_size(static_cast<int>(x_size), static_cast<int>(y_size));
  grid_map_ = std::make_shared<GridMap2D>(resolution_, map_size);
  grid_map_->setInflateRadius(inflate_values);

  kino_environment_ = std::make_shared<fast_planner::EDTEnvironment>(grid_map_);
  kinodynamic_astar_ = std::make_shared<fast_planner::KinodynamicAstar>();
  kinodynamic_astar_->setParam(resolution_, max_vel_, max_acc_);
  kinodynamic_astar_->setEnvironment(kino_environment_);
  kinodynamic_astar_->init();

  sdf_map_ = std::make_shared<SDFMap>();
  sdf_map_->initMap(x_size, y_size, 0.5, resolution_,
                    Eigen::Vector3d::Zero(), inflate_values);
  fast_backend_.reset(new BsplineReplanner);
  fast_backend_->initialize(max_vel_, max_acc_);
  // The SDF already includes the vehicle's single-circle radius. This extra
  // clearance is a small margin, scaled for the 2D ground robot.
  fast_backend_->setClearanceDistance(0.15);

}

void PlannerInterface::setPathPoint(const std::vector<PathPoint>& plan_traj) {
  global_plan_traj_ = plan_traj;
}

void PlannerInterface::setObstacles(const std::vector<ObstacleInfo>& obstacles) {
  if (!grid_map_) return;
  grid_map_->resetMap();
  for (const auto& obstacle : obstacles) {
    const Eigen::Vector2i index = grid_map_->worldToGrid(Eigen::Vector2d(obstacle.x, obstacle.y));
    grid_map_->setObstacle(index, true);
  }
  grid_map_->inflate();
}

void PlannerInterface::setCurrentVehiclePos(const PathPoint& cur_pose) {
  cur_pose_ = cur_pose;
  cur_pose_.theta = cur_pose.z;
}

void PlannerInterface::setGridMap(const PathPoint& cur_pose) {
  if (!grid_map_) return;
  grid_map_->setCurPose(cur_pose.x, cur_pose.y);
  if (kinodynamic_astar_) kinodynamic_astar_->init();
}

void PlannerInterface::makePlan() {
  plan_traj_results_.clear();
  search_paths_.clear();
  if (!kinodynamic_astar_ || !grid_map_ || global_plan_traj_.empty()) return;

  const auto& goal = global_plan_traj_.back();
  const Eigen::Vector3d start(cur_pose_.x, cur_pose_.y, 0.0);
  const Eigen::Vector3d goal_pos(goal.x, goal.y, 0.0);
  // Match the standalone planner's boundary conditions: start and stop, with
  // a small initial acceleration toward the destination. Heading is carried
  // separately in PathPoint::theta, estimated from the reference path.
  const Eigen::Vector3d start_vel = Eigen::Vector3d::Zero();
  const Eigen::Vector3d goal_vel(0.1 * std::cos(goal.theta),
                                 0.1 * std::sin(goal.theta), 0.0);
  const Eigen::Vector3d displacement = goal_pos - start;
  Eigen::Vector3d start_acc = Eigen::Vector3d::Zero();
  if (displacement.norm() > 1e-6) {
    start_acc = 0.5 * displacement.normalized();
  }

  kinodynamic_astar_->reset();
  const int result = kinodynamic_astar_->search(start, start_vel, start_acc,
                                                goal_pos, goal_vel, true);
  if (result != fast_planner::KinodynamicAstar::REACH_END &&
      result != fast_planner::KinodynamicAstar::NEAR_END) {
    return;
  }

  const auto points = kinodynamic_astar_->getKinoTraj(0.1);
  if (points.size() < 2) return;

  // Rebuild the Fast Planner ESDF around the current vehicle pose, then feed
  // raw obstacle cells so its inflation corresponds to the same single-circle
  // radius used by Kinodynamic A*.
  const Eigen::Vector3d robot_position(cur_pose_.x, cur_pose_.y, 0.25);
  sdf_map_->updateMapRolling(robot_position);
  for (const auto& obstacle : grid_map_->getObstaclePointCloud(false)) {
    sdf_map_->addLaserPoints(Eigen::Vector3d(obstacle.x(), obstacle.y(), 0.25), 1);
  }
  sdf_map_->startUpdateMapInfo(robot_position);

  std::vector<Eigen::Vector3d> seed_points;
  seed_points.reserve(points.size());
  for (const auto& p : points) seed_points.emplace_back(p.x(), p.y(), 0.25);

  std::vector<Eigen::Vector2d> path;
  std::vector<Eigen::Vector2d> seed_path;
  seed_path.reserve(points.size());
  for (const auto& p : points) seed_path.emplace_back(p.head<2>());
  bool accepted_optimized_path = false;
  if (points.size() >= 4) {
    std::vector<Eigen::Vector3d> endpoint_derivatives{
        start_vel, goal_vel, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
    fast_backend_->setInput(sdf_map_, 0.1, seed_points, endpoint_derivatives);
    fast_backend_->resetLambda2();

    NonUniformBspline optimized_trajectory;
    for (int attempt = 0; attempt < 5; ++attempt) {
      const bool dynamically_feasible = fast_backend_->optimize(true);
      optimized_trajectory = fast_backend_->getTraj();

      double begin = 0.0, end = 0.0;
      bool collision_free = optimized_trajectory.getControlPoint().rows() >= 4;
      if (collision_free) {
        optimized_trajectory.getRegion(begin, end);
        collision_free = end > begin;
      }
      if (collision_free) {
        path.clear();
        for (double t = begin; t < end; t += 0.05) {
          const Eigen::Vector3d p = optimized_trajectory.evaluateDeBoor(t);
          if (!p.allFinite() || grid_map_->getInflateOccupancy(p.head<2>())) {
            collision_free = false;
            break;
          }
          path.emplace_back(p.head<2>());
        }
        const Eigen::Vector3d p_end = optimized_trajectory.evaluateDeBoor(end);
        if (!p_end.allFinite() || grid_map_->getInflateOccupancy(p_end.head<2>())) {
          collision_free = false;
        } else if (collision_free) {
          path.emplace_back(p_end.head<2>());
        }
      }

      if (dynamically_feasible && collision_free && path.size() >= 2) {
        accepted_optimized_path = true;
        break;
      }
      fast_backend_->renewLambda2(0.05);
    }
  }

  if (!accepted_optimized_path) {
    path = seed_path;
  }

  path.reserve(points.size());
  plan_traj_results_.reserve(path.size());
  for (size_t i = 0; i < path.size(); ++i) {
    PathPoint point;
    point.x = static_cast<float>(path[i].x());
    point.y = static_cast<float>(path[i].y());
    if (i + 1 < path.size()) {
      point.theta = static_cast<float>(std::atan2(path[i + 1].y() - path[i].y(),
                                                  path[i + 1].x() - path[i].x()));
    } else {
      // The terminal state is near-stationary, so use the requested yaw.
      point.theta = goal.theta;
    }
    plan_traj_results_.push_back(point);
  }
  search_paths_.push_back(std::move(seed_path));
  if (accepted_optimized_path) search_paths_.push_back(path);
}

void PlannerInterface::getLocalPlanTrajResults(std::vector<PathPoint>& results) const {
  results = plan_traj_results_;
}

void PlannerInterface::getObstacles(std::vector<ObstacleInfo>& obstacles) const {
  if (!grid_map_) return;
  for (const auto& position : grid_map_->getObstaclePointCloud()) {
    ObstacleInfo obstacle;
    obstacle.x = static_cast<float>(position.x());
    obstacle.y = static_cast<float>(position.y());
    obstacles.push_back(obstacle);
  }
}

void PlannerInterface::getSearchPath(
    std::vector<std::vector<Eigen::Vector2d>>& paths) const {
  paths = search_paths_;
}

}  // namespace kinodynamic_astar_ros
