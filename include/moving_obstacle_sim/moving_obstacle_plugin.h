#ifndef MOVING_OBSTACLE_SIM_MOVING_OBSTACLE_PLUGIN_H_
#define MOVING_OBSTACLE_SIM_MOVING_OBSTACLE_PLUGIN_H_

#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>

namespace moving_obstacle_sim
{

enum class MotionType
{
  kStatic,
  kLine,
  kSine,
  kCircle,
  kEllipse,
  kPolygon
};

enum class MotionMode
{
  kLoop,
  kPingPong,
  kOnce
};

struct PolySegment
{
  ignition::math::Vector2d start;
  ignition::math::Vector2d dir;
  double length{0.0};
  double cumulative_end{0.0};
};

class MovingObstaclePlugin : public gazebo::ModelPlugin
{
public:
  MovingObstaclePlugin();
  ~MovingObstaclePlugin() override = default;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  void OnUpdate(const gazebo::common::UpdateInfo& info);
  bool ConfigureFromSdf(sdf::ElementPtr sdf);
  bool ValidateMotionConfig();
  void AdvanceProgress(double& progress, double delta, double min_value, double max_value);
  bool ParsePolygonPoints(const std::string& raw_points);

  bool ComputeLinePose(double dt, ignition::math::Vector2d& xy, ignition::math::Vector2d& tangent);
  bool ComputeSinePose(double dt, ignition::math::Vector2d& xy, ignition::math::Vector2d& tangent);
  bool ComputeCirclePose(double dt, ignition::math::Vector2d& xy, ignition::math::Vector2d& tangent);
  bool ComputeEllipsePose(double dt, ignition::math::Vector2d& xy, ignition::math::Vector2d& tangent);
  bool ComputePolygonPose(double dt, ignition::math::Vector2d& xy, ignition::math::Vector2d& tangent);

  bool ApplyBounds(ignition::math::Vector2d& xy, ignition::math::Vector2d& tangent);
  void DowngradeToStatic(const std::string& reason);

  gazebo::physics::ModelPtr model_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo::common::Time last_sim_time_;

  ignition::math::Pose3d initial_pose_;
  double initial_roll_{0.0};
  double initial_pitch_{0.0};
  double current_yaw_{0.0};
  double fixed_z_{0.0};

  MotionType motion_type_{MotionType::kStatic};
  MotionMode mode_{MotionMode::kPingPong};

  bool motion_enabled_{false};
  bool align_yaw_{false};
  bool z_lock_{true};

  double speed_{0.5};
  int travel_dir_{1};

  bool bounds_enabled_{false};
  double x_min_{-1e9};
  double x_max_{1e9};
  double y_min_{-1e9};
  double y_max_{1e9};

  ignition::math::Vector2d line_p0_{0.0, 0.0};
  ignition::math::Vector2d line_p1_{0.0, 0.0};
  double line_length_{0.0};

  std::string sine_axis_{"x"};
  double sine_min_{-1.0};
  double sine_max_{1.0};
  double sine_center_{0.0};
  double sine_amplitude_{0.5};
  double sine_wavelength_{2.0};
  double sine_phase_{0.0};
  double sine_path_length_{0.0};

  ignition::math::Vector2d circle_center_{0.0, 0.0};
  double circle_radius_{1.0};
  double circle_theta0_{0.0};
  int circle_base_sign_{1};

  ignition::math::Vector2d ellipse_center_{0.0, 0.0};
  double ellipse_a_{1.5};
  double ellipse_b_{1.0};
  double ellipse_yaw_{0.0};
  double ellipse_theta0_{0.0};
  int ellipse_base_sign_{1};

  std::vector<ignition::math::Vector2d> polygon_points_;
  std::vector<PolySegment> polygon_segments_;
  bool polygon_close_loop_{true};
  double polygon_total_length_{0.0};

  double scalar_progress_{0.0};
  double phase_progress_{0.0};
};

}  // namespace moving_obstacle_sim

#endif  // MOVING_OBSTACLE_SIM_MOVING_OBSTACLE_PLUGIN_H_
