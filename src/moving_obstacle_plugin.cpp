#include "moving_obstacle_sim/moving_obstacle_plugin.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include <ros/ros.h>

namespace
{
constexpr double kEps = 1e-6;
constexpr double kTwoPi = 2.0 * M_PI;

std::string ToLower(std::string input)
{
  std::transform(input.begin(), input.end(), input.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return input;
}

template <typename T>
T GetOrDefault(sdf::ElementPtr sdf, const std::string& key, const T& default_value)
{
  if (sdf && sdf->HasElement(key))
  {
    return sdf->Get<T>(key);
  }
  return default_value;
}

bool ParseVec2String(const std::string& text, ignition::math::Vector2d& out)
{
  std::istringstream iss(text);
  double x = 0.0;
  double y = 0.0;
  if (!(iss >> x >> y))
  {
    return false;
  }
  out.Set(x, y);
  return true;
}

double WrapRange(double value, double min_value, double max_value)
{
  const double range = max_value - min_value;
  if (range <= kEps)
  {
    return min_value;
  }
  while (value > max_value)
  {
    value -= range;
  }
  while (value < min_value)
  {
    value += range;
  }
  return value;
}

}  // namespace

namespace moving_obstacle_sim
{

MovingObstaclePlugin::MovingObstaclePlugin() = default;

void MovingObstaclePlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;
  if (!model_)
  {
    ROS_ERROR("[moving_obstacle_plugin] Invalid model pointer.");
    return;
  }

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = nullptr;
    ros::init(argc, argv, "moving_obstacle_plugin", ros::init_options::NoSigintHandler);
  }

  initial_pose_ = model_->WorldPose();
  initial_roll_ = initial_pose_.Rot().Roll();
  initial_pitch_ = initial_pose_.Rot().Pitch();
  current_yaw_ = initial_pose_.Rot().Yaw();
  fixed_z_ = initial_pose_.Pos().Z();

  ConfigureFromSdf(sdf);
  ValidateMotionConfig();

  if (!motion_enabled_)
  {
    ROS_INFO_STREAM("[moving_obstacle_plugin] Model '" << model_->GetName()
                    << "' configured as static obstacle.");
    return;
  }

  last_sim_time_ = model_->GetWorld()->SimTime();
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&MovingObstaclePlugin::OnUpdate, this, std::placeholders::_1));

  ROS_INFO_STREAM("[moving_obstacle_plugin] Model '" << model_->GetName() << "' motion type ready.");
}

bool MovingObstaclePlugin::ConfigureFromSdf(sdf::ElementPtr sdf)
{
  motion_enabled_ = GetOrDefault<bool>(sdf, "enabled", true);
  speed_ = GetOrDefault<double>(sdf, "speed", 0.5);
  align_yaw_ = GetOrDefault<bool>(sdf, "align_yaw", false);
  z_lock_ = GetOrDefault<bool>(sdf, "z_lock", true);

  const std::string mode_text = ToLower(GetOrDefault<std::string>(sdf, "mode", "ping_pong"));
  if (mode_text == "loop")
  {
    mode_ = MotionMode::kLoop;
  }
  else if (mode_text == "once")
  {
    mode_ = MotionMode::kOnce;
  }
  else
  {
    mode_ = MotionMode::kPingPong;
  }

  bounds_enabled_ = GetOrDefault<bool>(sdf, "bounds_enabled", false);
  x_min_ = GetOrDefault<double>(sdf, "x_min", -1e9);
  x_max_ = GetOrDefault<double>(sdf, "x_max", 1e9);
  y_min_ = GetOrDefault<double>(sdf, "y_min", -1e9);
  y_max_ = GetOrDefault<double>(sdf, "y_max", 1e9);

  if (x_min_ > x_max_)
  {
    std::swap(x_min_, x_max_);
  }
  if (y_min_ > y_max_)
  {
    std::swap(y_min_, y_max_);
  }

  const std::string type = ToLower(GetOrDefault<std::string>(sdf, "type", "static"));
  if (type == "line")
  {
    motion_type_ = MotionType::kLine;
  }
  else if (type == "sine")
  {
    motion_type_ = MotionType::kSine;
  }
  else if (type == "circle")
  {
    motion_type_ = MotionType::kCircle;
  }
  else if (type == "ellipse")
  {
    motion_type_ = MotionType::kEllipse;
  }
  else if (type == "polygon")
  {
    motion_type_ = MotionType::kPolygon;
  }
  else
  {
    motion_type_ = MotionType::kStatic;
  }

  if (motion_type_ == MotionType::kStatic)
  {
    motion_enabled_ = false;
    return true;
  }

  const ignition::math::Vector2d initial_xy(initial_pose_.Pos().X(), initial_pose_.Pos().Y());

  line_p0_ = initial_xy + ignition::math::Vector2d(-1.0, 0.0);
  line_p1_ = initial_xy + ignition::math::Vector2d(1.0, 0.0);
  if (sdf && sdf->HasElement("line_p0"))
  {
    ParseVec2String(sdf->Get<std::string>("line_p0"), line_p0_);
  }
  if (sdf && sdf->HasElement("line_p1"))
  {
    ParseVec2String(sdf->Get<std::string>("line_p1"), line_p1_);
  }
  line_length_ = (line_p1_ - line_p0_).Length();

  sine_axis_ = ToLower(GetOrDefault<std::string>(sdf, "sine_axis", "x"));
  sine_min_ = (sine_axis_ == "y") ? GetOrDefault<double>(sdf, "sine_y_min", initial_xy.Y() - 1.0)
                                    : GetOrDefault<double>(sdf, "sine_x_min", initial_xy.X() - 1.0);
  sine_max_ = (sine_axis_ == "y") ? GetOrDefault<double>(sdf, "sine_y_max", initial_xy.Y() + 1.0)
                                    : GetOrDefault<double>(sdf, "sine_x_max", initial_xy.X() + 1.0);
  if (sine_min_ > sine_max_)
  {
    std::swap(sine_min_, sine_max_);
  }
  sine_center_ = GetOrDefault<double>(sdf, "sine_center", (sine_axis_ == "y") ? initial_xy.X() : initial_xy.Y());
  sine_amplitude_ = GetOrDefault<double>(sdf, "sine_amplitude", 0.5);
  sine_wavelength_ = GetOrDefault<double>(sdf, "sine_wavelength", 2.0);
  sine_phase_ = GetOrDefault<double>(sdf, "sine_phase", 0.0);
  sine_path_length_ = sine_max_ - sine_min_;

  circle_center_ = initial_xy;
  if (sdf && sdf->HasElement("circle_center"))
  {
    ParseVec2String(sdf->Get<std::string>("circle_center"), circle_center_);
  }
  circle_radius_ = GetOrDefault<double>(sdf, "circle_radius", 1.0);
  circle_theta0_ = GetOrDefault<double>(sdf, "circle_theta0", 0.0);
  circle_base_sign_ = GetOrDefault<bool>(sdf, "circle_clockwise", false) ? -1 : 1;

  ellipse_center_ = initial_xy;
  if (sdf && sdf->HasElement("ellipse_center"))
  {
    ParseVec2String(sdf->Get<std::string>("ellipse_center"), ellipse_center_);
  }
  ellipse_a_ = GetOrDefault<double>(sdf, "ellipse_a", 1.5);
  ellipse_b_ = GetOrDefault<double>(sdf, "ellipse_b", 1.0);
  ellipse_yaw_ = GetOrDefault<double>(sdf, "ellipse_yaw", 0.0);
  ellipse_theta0_ = GetOrDefault<double>(sdf, "ellipse_theta0", 0.0);
  ellipse_base_sign_ = GetOrDefault<bool>(sdf, "ellipse_clockwise", false) ? -1 : 1;

  polygon_close_loop_ = GetOrDefault<bool>(sdf, "polygon_close_loop", true);
  polygon_points_.clear();
  polygon_segments_.clear();
  polygon_total_length_ = 0.0;
  if (sdf && sdf->HasElement("polygon_points"))
  {
    ParsePolygonPoints(sdf->Get<std::string>("polygon_points"));
  }

  travel_dir_ = 1;
  scalar_progress_ = 0.0;
  phase_progress_ = 0.0;
  return true;
}

bool MovingObstaclePlugin::ValidateMotionConfig()
{
  if (!motion_enabled_)
  {
    return true;
  }

  if (speed_ <= kEps)
  {
    DowngradeToStatic("speed <= 0");
    return false;
  }

  switch (motion_type_)
  {
    case MotionType::kLine:
      if (line_length_ <= kEps)
      {
        DowngradeToStatic("line length is zero");
        return false;
      }
      break;
    case MotionType::kSine:
      if (sine_path_length_ <= kEps)
      {
        DowngradeToStatic("sine axis range is zero");
        return false;
      }
      if (std::fabs(sine_wavelength_) <= kEps)
      {
        DowngradeToStatic("sine wavelength <= 0");
        return false;
      }
      break;
    case MotionType::kCircle:
      if (circle_radius_ <= kEps)
      {
        DowngradeToStatic("circle radius <= 0");
        return false;
      }
      break;
    case MotionType::kEllipse:
      if (ellipse_a_ <= kEps || ellipse_b_ <= kEps)
      {
        DowngradeToStatic("ellipse a <= 0 or b <= 0");
        return false;
      }
      break;
    case MotionType::kPolygon:
      if (polygon_points_.size() < 2 || polygon_total_length_ <= kEps)
      {
        DowngradeToStatic("polygon requires at least two valid points");
        return false;
      }
      break;
    case MotionType::kStatic:
    default:
      motion_enabled_ = false;
      break;
  }

  return motion_enabled_;
}

void MovingObstaclePlugin::DowngradeToStatic(const std::string& reason)
{
  motion_enabled_ = false;
  motion_type_ = MotionType::kStatic;
  ROS_WARN_STREAM("[moving_obstacle_plugin] Model '" << model_->GetName()
                  << "' downgraded to static: " << reason);
}

void MovingObstaclePlugin::AdvanceProgress(double& progress, double delta, double min_value, double max_value)
{
  if (delta <= kEps)
  {
    return;
  }

  if (max_value - min_value <= kEps)
  {
    progress = min_value;
    return;
  }

  if (mode_ == MotionMode::kLoop)
  {
    progress += static_cast<double>(travel_dir_) * delta;
    progress = WrapRange(progress, min_value, max_value);
    return;
  }

  if (mode_ == MotionMode::kPingPong)
  {
    progress += static_cast<double>(travel_dir_) * delta;
    if (progress > max_value)
    {
      progress = max_value;
      travel_dir_ = -1;
    }
    else if (progress < min_value)
    {
      progress = min_value;
      travel_dir_ = 1;
    }
    return;
  }

  progress += static_cast<double>(travel_dir_) * delta;
  if (progress >= max_value)
  {
    progress = max_value;
    motion_enabled_ = false;
  }
  else if (progress <= min_value)
  {
    progress = min_value;
    motion_enabled_ = false;
  }
}

bool MovingObstaclePlugin::ParsePolygonPoints(const std::string& raw_points)
{
  std::vector<ignition::math::Vector2d> points;
  std::stringstream all(raw_points);
  std::string token;
  while (std::getline(all, token, ';'))
  {
    if (token.empty())
    {
      continue;
    }
    ignition::math::Vector2d p;
    if (!ParseVec2String(token, p))
    {
      ROS_WARN_STREAM("[moving_obstacle_plugin] Ignore invalid polygon point token: '" << token << "'");
      continue;
    }
    points.push_back(p);
  }

  if (points.size() < 2)
  {
    return false;
  }

  if (polygon_close_loop_)
  {
    if ((points.front() - points.back()).Length() > kEps)
    {
      points.push_back(points.front());
    }
  }

  polygon_points_ = points;
  polygon_segments_.clear();
  polygon_total_length_ = 0.0;

  for (size_t i = 0; i + 1 < polygon_points_.size(); ++i)
  {
    const ignition::math::Vector2d diff = polygon_points_[i + 1] - polygon_points_[i];
    const double length = diff.Length();
    if (length <= kEps)
    {
      continue;
    }

    PolySegment segment;
    segment.start = polygon_points_[i];
    segment.dir = diff / length;
    segment.length = length;
    polygon_total_length_ += length;
    segment.cumulative_end = polygon_total_length_;
    polygon_segments_.push_back(segment);
  }

  return !polygon_segments_.empty();
}

bool MovingObstaclePlugin::ComputeLinePose(double dt, ignition::math::Vector2d& xy, ignition::math::Vector2d& tangent)
{
  AdvanceProgress(scalar_progress_, speed_ * dt, 0.0, line_length_);
  const ignition::math::Vector2d dir = (line_p1_ - line_p0_) / std::max(line_length_, kEps);
  xy = line_p0_ + dir * scalar_progress_;
  tangent = dir * static_cast<double>(travel_dir_);
  return true;
}

bool MovingObstaclePlugin::ComputeSinePose(double dt, ignition::math::Vector2d& xy, ignition::math::Vector2d& tangent)
{
  AdvanceProgress(scalar_progress_, speed_ * dt, 0.0, sine_path_length_);

  const double coord = sine_min_ + scalar_progress_;
  const double k = kTwoPi / sine_wavelength_;

  if (sine_axis_ == "y")
  {
    const double y = coord;
    const double arg = k * y + sine_phase_;
    const double x = sine_center_ + sine_amplitude_ * std::sin(arg);
    xy.Set(x, y);
    tangent.Set(sine_amplitude_ * k * std::cos(arg), 1.0);
  }
  else
  {
    const double x = coord;
    const double arg = k * x + sine_phase_;
    const double y = sine_center_ + sine_amplitude_ * std::sin(arg);
    xy.Set(x, y);
    tangent.Set(1.0, sine_amplitude_ * k * std::cos(arg));
  }

  if (tangent.Length() > kEps)
  {
    tangent.Normalize();
  }
  tangent *= static_cast<double>(travel_dir_);
  return true;
}

bool MovingObstaclePlugin::ComputeCirclePose(double dt, ignition::math::Vector2d& xy, ignition::math::Vector2d& tangent)
{
  const double delta_phase = speed_ * dt / std::max(circle_radius_, kEps);
  AdvanceProgress(phase_progress_, delta_phase, 0.0, kTwoPi);

  const double theta = circle_theta0_ + static_cast<double>(circle_base_sign_) * phase_progress_;
  const double c = std::cos(theta);
  const double s = std::sin(theta);

  xy.Set(circle_center_.X() + circle_radius_ * c, circle_center_.Y() + circle_radius_ * s);
  tangent.Set(-s, c);
  tangent *= static_cast<double>(circle_base_sign_ * travel_dir_);
  return true;
}

bool MovingObstaclePlugin::ComputeEllipsePose(double dt, ignition::math::Vector2d& xy, ignition::math::Vector2d& tangent)
{
  const double current_theta = ellipse_theta0_ + static_cast<double>(ellipse_base_sign_) * phase_progress_;
  const double ds_dtheta = std::sqrt(std::pow(ellipse_a_ * std::sin(current_theta), 2.0) +
                                     std::pow(ellipse_b_ * std::cos(current_theta), 2.0));

  const double delta_phase = speed_ * dt / std::max(ds_dtheta, kEps);
  AdvanceProgress(phase_progress_, delta_phase, 0.0, kTwoPi);

  const double theta = ellipse_theta0_ + static_cast<double>(ellipse_base_sign_) * phase_progress_;
  const double cos_t = std::cos(theta);
  const double sin_t = std::sin(theta);
  const double cos_y = std::cos(ellipse_yaw_);
  const double sin_y = std::sin(ellipse_yaw_);

  const double lx = ellipse_a_ * cos_t;
  const double ly = ellipse_b_ * sin_t;

  xy.Set(ellipse_center_.X() + cos_y * lx - sin_y * ly,
         ellipse_center_.Y() + sin_y * lx + cos_y * ly);

  ignition::math::Vector2d dtheta(-ellipse_a_ * sin_t, ellipse_b_ * cos_t);
  ignition::math::Vector2d rotated(cos_y * dtheta.X() - sin_y * dtheta.Y(),
                                   sin_y * dtheta.X() + cos_y * dtheta.Y());

  if (rotated.Length() > kEps)
  {
    rotated.Normalize();
  }
  tangent = rotated * static_cast<double>(ellipse_base_sign_ * travel_dir_);
  return true;
}

bool MovingObstaclePlugin::ComputePolygonPose(double dt, ignition::math::Vector2d& xy, ignition::math::Vector2d& tangent)
{
  AdvanceProgress(scalar_progress_, speed_ * dt, 0.0, polygon_total_length_);

  double prev_cumulative = 0.0;
  for (const auto& segment : polygon_segments_)
  {
    if (scalar_progress_ <= segment.cumulative_end + kEps)
    {
      const double local_s = std::max(0.0, std::min(segment.length, scalar_progress_ - prev_cumulative));
      xy = segment.start + segment.dir * local_s;
      tangent = segment.dir * static_cast<double>(travel_dir_);
      return true;
    }
    prev_cumulative = segment.cumulative_end;
  }

  const auto& last = polygon_segments_.back();
  xy = last.start + last.dir * last.length;
  tangent = last.dir * static_cast<double>(travel_dir_);
  return true;
}

bool MovingObstaclePlugin::ApplyBounds(ignition::math::Vector2d& xy, ignition::math::Vector2d& tangent)
{
  if (!bounds_enabled_)
  {
    return false;
  }

  bool hit = false;
  if (xy.X() < x_min_)
  {
    xy.X(x_min_);
    hit = true;
  }
  else if (xy.X() > x_max_)
  {
    xy.X(x_max_);
    hit = true;
  }

  if (xy.Y() < y_min_)
  {
    xy.Y(y_min_);
    hit = true;
  }
  else if (xy.Y() > y_max_)
  {
    xy.Y(y_max_);
    hit = true;
  }

  if (hit)
  {
    travel_dir_ *= -1;
    tangent *= -1.0;
  }

  return hit;
}

void MovingObstaclePlugin::OnUpdate(const gazebo::common::UpdateInfo& info)
{
  if (!motion_enabled_)
  {
    return;
  }

  const gazebo::common::Time now = info.simTime;
  const double dt = (now - last_sim_time_).Double();
  last_sim_time_ = now;

  if (dt <= 0.0)
  {
    return;
  }

  ignition::math::Vector2d xy;
  ignition::math::Vector2d tangent(1.0, 0.0);
  bool ok = false;

  switch (motion_type_)
  {
    case MotionType::kLine:
      ok = ComputeLinePose(dt, xy, tangent);
      break;
    case MotionType::kSine:
      ok = ComputeSinePose(dt, xy, tangent);
      break;
    case MotionType::kCircle:
      ok = ComputeCirclePose(dt, xy, tangent);
      break;
    case MotionType::kEllipse:
      ok = ComputeEllipsePose(dt, xy, tangent);
      break;
    case MotionType::kPolygon:
      ok = ComputePolygonPose(dt, xy, tangent);
      break;
    case MotionType::kStatic:
    default:
      motion_enabled_ = false;
      return;
  }

  if (!ok)
  {
    DowngradeToStatic("failed to compute pose");
    return;
  }

  ApplyBounds(xy, tangent);

  if (align_yaw_ && tangent.Length() > kEps)
  {
    current_yaw_ = std::atan2(tangent.Y(), tangent.X());
  }

  const double z = z_lock_ ? fixed_z_ : model_->WorldPose().Pos().Z();
  ignition::math::Pose3d pose;
  pose.Pos().Set(xy.X(), xy.Y(), z);
  pose.Rot() = ignition::math::Quaterniond(initial_roll_, initial_pitch_, current_yaw_);

  model_->SetLinearVel(ignition::math::Vector3d::Zero);
  model_->SetAngularVel(ignition::math::Vector3d::Zero);
  model_->SetWorldPose(pose);
}

}  // namespace moving_obstacle_sim

GZ_REGISTER_MODEL_PLUGIN(moving_obstacle_sim::MovingObstaclePlugin)
