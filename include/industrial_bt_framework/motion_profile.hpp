#pragma once

#include <string>
#include <unordered_map>

namespace industrial_bt_framework {

// Per-segment motion tuning parameters.
// Loaded from motion_profiles.yaml at bt_runner_node startup.
struct MotionProfile
{
  // Free-space (OMPL) planning
  double planning_time_sec          = 4.0;
  int    num_planning_attempts      = 30;
  int    max_free_space_retries     = 15;
  double velocity_scaling           = 0.20;
  double acceleration_scaling       = 0.20;
  double goal_position_tolerance    = 0.003;
  double goal_orientation_tolerance = 0.03;
  double joint_target_tolerance     = 0.02;

  // Cartesian motion (used by MoveIt backend)
  double cartesian_primary_step_m     = 0.010;
  double cartesian_fallback_step_m    = 0.005;
  double cartesian_eef_step           = 0.003;
  double cartesian_jump_threshold     = 0.0;
  double cartesian_min_fraction       = 0.999;
  double cartesian_final_position_tol = 0.003;

  // Vendor-plugin Cartesian hint (e.g. Fairino movel speed_percent)
  double movel_speed_percent = 50.0;

  // Timing
  int    scene_update_wait_ms   = 800;
  int    retry_sleep_ms         = 150;
  double world_spawn_safety_z_m = 0.001;
};

using MotionProfileRegistry = std::unordered_map<std::string, MotionProfile>;

}  // namespace industrial_bt_framework
