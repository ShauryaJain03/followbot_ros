#include <gazebo_ros_actor_plugin/gazebo_ros_actor_command.h>

#include <algorithm>
#include <chrono>

using namespace gazebo_ros_actor_plugin;

GazeboRosActorCommand::GazeboRosActorCommand()
  : actorEntity_(gz::sim::kNullEntity),
    animationFactor_(4.0),
    lastUpdate_(std::chrono::steady_clock::duration::zero()),
    lastHumanPublish_(std::chrono::steady_clock::duration::zero()),
    humanPublishPeriod_(
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(0.1))),
    followMode_("velocity"),
    targetVel_(gz::math::Pose3d::Zero),
    linVelocity_(1.0),
    angVelocity_(IGN_DTOR(10)),
    idx_(0),
    linTolerance_(0.1),
    angTolerance_(IGN_DTOR(5)),
    defaultRotation_(M_PI/2),
    pathCompletedLogged_(false),
    publishHumanState_(true),
    humanPathDirty_(false),
    terrainLoaded_(false),
    terrainZOffset_(0.0),
    fallbackZ_(0.0) {
  this->humanPoseTopic_ = "/human_pose";
  this->humanPathTopic_ = "/human_path";
  this->humanFrameId_ = "map";
}

void GazeboRosActorCommand::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/) {

  this->actorEntity_ = _entity;

  auto actorComp = _ecm.Component<gz::sim::components::Actor>(this->actorEntity_);
  if (!actorComp)
  {
    ignerr << "Entity [" << _entity << "] is not an actor." << std::endl;
    return;
  }

  if (_sdf->HasElement("follow_mode")) {
    this->followMode_ = _sdf->Get<std::string>("follow_mode");
  }
  if (_sdf->HasElement("vel_topic")) {
    this->velTopic_ = _sdf->Get<std::string>("vel_topic");
  }
  if (_sdf->HasElement("path_topic")) {
    this->pathTopic_ = _sdf->Get<std::string>("path_topic");
  }
  if (_sdf->HasElement("linear_tolerance")) {
    this->linTolerance_ = _sdf->Get<double>("linear_tolerance");
  }
  if (_sdf->HasElement("linear_velocity")) {
    this->linVelocity_ = _sdf->Get<double>("linear_velocity");
  }
  if (_sdf->HasElement("angular_tolerance")) {
    this->angTolerance_ = _sdf->Get<double>("angular_tolerance");
  }
  if (_sdf->HasElement("angular_velocity")) {
    this->angVelocity_ = _sdf->Get<double>("angular_velocity");
  }
  if (_sdf->HasElement("animation_factor")) {
    this->animationFactor_ = _sdf->Get<double>("animation_factor");
  }
  if (_sdf->HasElement("default_rotation")) {
    this->defaultRotation_ = _sdf->Get<double>("default_rotation");
  }

  this->ConfigureRosPublishers(_sdf);

  if (_sdf->HasElement("terrain")) {
    auto sdfCopy = _sdf->Clone();
    auto terrainElem = sdfCopy->GetElement("terrain");
    while (terrainElem)
    {
      this->LoadTerrainMeshes(terrainElem);
      terrainElem = terrainElem->GetNextElement("terrain");
    }
  }

  std::string animationName;

  // If animation not provided, use first one from SDF
  if (!_sdf->HasElement("animation"))
  {
    if (actorComp->Data().AnimationCount() < 1)
    {
      ignerr << "Actor SDF doesn't have any animations." << std::endl;
      return;
    }

    animationName = actorComp->Data().AnimationByIndex(0)->Name();
  }
  else
  {
    animationName = _sdf->Get<std::string>("animation");
  }

  if (animationName.empty())
  {
    ignerr << "Can't find actor's animation name." << std::endl;
    return;
  }

  auto animationNameComp = _ecm.Component<gz::sim::components::AnimationName>(_entity);
  if (nullptr == animationNameComp)
  {
    _ecm.CreateComponent(_entity, gz::sim::components::AnimationName(animationName));
  }
  else
  {
    *animationNameComp = gz::sim::components::AnimationName(animationName);
  }
  // Mark as a one-time-change so that the change is propagated to the GUI
  _ecm.SetChanged(_entity,
    gz::sim::components::AnimationName::typeId, gz::sim::ComponentState::OneTimeChange);

  // Set custom animation time from this plugin
  auto animTimeComp = _ecm.Component<gz::sim::components::AnimationTime>(_entity);
  if (nullptr == animTimeComp)
  {
    _ecm.CreateComponent(_entity, gz::sim::components::AnimationTime());
  }

  gz::math::Pose3d initialPose;
  auto poseComp = _ecm.Component<gz::sim::components::Pose>(_entity);
  if (nullptr == poseComp)
  {
    _ecm.CreateComponent(_entity, gz::sim::components::Pose(
      gz::math::Pose3d::Zero));
  }
  else
  {
    initialPose = poseComp->Data();
    this->fallbackZ_ = initialPose.Pos().Z();

    // We'll be setting the actor's X/Y pose with respect to the world. So we
    // zero the current values.
    auto newPose = initialPose;
    newPose.Pos().X(0);
    newPose.Pos().Y(0);
    *poseComp = gz::sim::components::Pose(newPose);
  }

  // Having a trajectory pose prevents the actor from moving with the
  // SDF script
  auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(_entity);
  if (nullptr == trajPoseComp)
  {
    // Preserve the actor's initial Z so world files can place the actor on
    // the intended surface while this plugin controls only planar motion.
    if (this->terrainLoaded_)
    {
      auto terrainZ = this->TerrainHeight(initialPose.Pos().X(), initialPose.Pos().Y());
      if (terrainZ.has_value())
      {
        this->terrainZOffset_ = initialPose.Pos().Z() - terrainZ.value();
      }
    }
    _ecm.CreateComponent(_entity, gz::sim::components::TrajectoryPose(initialPose));
  }

  if (!this->node_.Subscribe(this->velTopic_, &GazeboRosActorCommand::VelCallback, this)) {
    ignerr << "Failed to subscribe to velocity topic: " << this->velTopic_ << std::endl;
  }

  if (!this->node_.Subscribe(this->pathTopic_, &GazeboRosActorCommand::PathCallback, this)) {
    ignerr << "Failed to subscribe to path topic: " << this->pathTopic_ << std::endl;
  }

  this->lastUpdate_ = std::chrono::steady_clock::duration::zero();
}

void GazeboRosActorCommand::VelCallback(const gz::msgs::Twist &msg) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  ignition::math::Vector3d velCmd;
  velCmd.X() = msg.linear().x();
  velCmd.Z() = msg.angular().z();
  this->cmdQueue_.push(velCmd);
}

void GazeboRosActorCommand::PathCallback(const gz::msgs::Pose_V &msg) {
  std::lock_guard<std::mutex> lock(this->mutex_);

  std::vector<gz::math::Vector3d> poses;

  for (int i = 0; i < msg.pose_size(); ++i) {
    const auto& pose = msg.pose(i);
    double x = pose.position().x();
    double y = pose.position().y();

    gz::math::Quaterniond quat(
      pose.orientation().w(),
      pose.orientation().x(),
      pose.orientation().y(),
      pose.orientation().z()
    );
    double yaw = quat.Euler().Z();

    poses.push_back(ignition::math::Vector3d(x, y, yaw));
  }

  if (!poses.empty()) {
    this->pathQueue_.push(poses);
    ignmsg << "New path received with " << poses.size() << " waypoints" << std::endl;
  } else {
    ignwarn << "Received empty path" << std::endl;
  }
}

void GazeboRosActorCommand::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) {

  IGN_PROFILE("GazeboRosActorCommand::PreUpdate");

  std::chrono::duration<double> dt = _info.simTime - this->lastUpdate_;
  this->lastUpdate_ = _info.simTime;

  auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(this->actorEntity_);
  auto actorPose = trajPoseComp->Data();
  auto currentPose = actorPose;

  gz::math::Vector3d rpy = currentPose.Rot().Euler();

  gz::math::Pose3d newPose = currentPose;
  double distanceTraveled = 0.0;

  if (this->followMode_ == "path") {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (!this->pathQueue_.empty()) {
      this->targetPoses_ = this->pathQueue_.front();
      this->pathQueue_.pop();

      this->idx_ = 0;
      if (!this->targetPoses_.empty()) {
        this->targetPose_ = this->targetPoses_.at(this->idx_);
        this->pathCompletedLogged_ = false;
        this->humanPathDirty_ = true;
        ignmsg << "New path loaded with " << this->targetPoses_.size()
          << " waypoints" << std::endl;
      }
    }

    if (this->humanPathDirty_) {
      this->PublishHumanPath(_info.simTime);
      this->humanPathDirty_ = false;
    }

    if (this->targetPoses_.empty() || this->idx_ >= static_cast<int>(this->targetPoses_.size())) {
      this->PublishHumanPose(newPose, _info.simTime);
      this->lastUpdate_ = _info.simTime;
      return;
    }

    gz::math::Vector2d targetPos2d(this->targetPose_.X(), this->targetPose_.Y());
    gz::math::Vector2d currentPos2d(currentPose.Pos().X(), currentPose.Pos().Y());
    gz::math::Vector2d pos = targetPos2d - currentPos2d;
    double distance = pos.Length();

    if (distance < this->linTolerance_) {
      if (this->idx_ < static_cast<int>(this->targetPoses_.size()) - 1) {
        this->ChooseNewTarget();
        pos.X() = this->targetPose_.X() - currentPose.Pos().X();
        pos.Y() = this->targetPose_.Y() - currentPose.Pos().Y();
      } else {
        if (!this->pathCompletedLogged_) {
          ignmsg << "Path completed - all waypoints reached" << std::endl;
          this->pathCompletedLogged_ = true;
        }
        pos.X() = 0;
        pos.Y() = 0;
      }
    }

    if (pos.Length() != 0) {
      pos = pos / pos.Length();
    }

    double targetYaw = std::atan2(pos.Y(), pos.X());

    newPose.Rot() = gz::math::Quaterniond(0, 0, targetYaw);

    if (pos.Length() != 0) {
      newPose.Pos().X() += pos.X() * this->linVelocity_ * dt.count();
      newPose.Pos().Y() += pos.Y() * this->linVelocity_ * dt.count();
      distanceTraveled = (pos * this->linVelocity_ * dt.count()).Length();
    }

  } else if (this->followMode_ == "velocity") {
    std::lock_guard<std::mutex> lock(this->mutex_);

    if (!this->cmdQueue_.empty()) {
      gz::math::Vector3d vel = this->cmdQueue_.front();
      this->cmdQueue_.pop();

      this->targetVel_.Pos().X() = vel.X();
      this->targetVel_.Rot() = gz::math::Quaterniond(0, 0, vel.Z());
    }

    if (std::abs(this->targetVel_.Pos().X()) > 0.001 ||
        std::abs(this->targetVel_.Rot().Euler().Z()) > 0.001) {
      double dx = this->targetVel_.Pos().X() *
        std::cos(currentPose.Rot().Euler().Z()) * dt.count();
      double dy = this->targetVel_.Pos().X() *
        std::sin(currentPose.Rot().Euler().Z()) * dt.count();

      newPose.Pos().X() += dx;
      newPose.Pos().Y() += dy;

      double newYaw = rpy.Z() + this->targetVel_.Rot().Euler().Z() * dt.count();
      newPose.Rot() = gz::math::Quaterniond(0, 0, newYaw);

      distanceTraveled = std::sqrt(dx * dx + dy * dy);
    } else {
      this->targetVel_ = gz::math::Pose3d::Zero;
    }
  }

  if (this->terrainLoaded_)
  {
    auto terrainZ = this->TerrainHeight(newPose.Pos().X(), newPose.Pos().Y());
    if (terrainZ.has_value())
    {
      newPose.Pos().Z(terrainZ.value() + this->terrainZOffset_);
    }
    else
    {
      newPose.Pos().Z(this->fallbackZ_);
    }
  }

  *trajPoseComp = gz::sim::components::TrajectoryPose(newPose);

  _ecm.SetChanged(
    this->actorEntity_,
    gz::sim::components::TrajectoryPose::typeId,
    gz::sim::ComponentState::OneTimeChange);

  // Update actor bone trajectories based on animation time
  auto animTimeComp = _ecm.Component<gz::sim::components::AnimationTime>(this->actorEntity_);

  if (distanceTraveled > 0.0001) {
    auto animTime = animTimeComp->Data() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(distanceTraveled * this->animationFactor_)
      );

    *animTimeComp = gz::sim::components::AnimationTime(animTime);

    _ecm.SetChanged(
      this->actorEntity_,
      gz::sim::components::AnimationTime::typeId,
      gz::sim::ComponentState::OneTimeChange);
  }

  this->PublishHumanPose(newPose, _info.simTime);
}

void GazeboRosActorCommand::ChooseNewTarget() {
  this->idx_++;

  if (this->idx_ < static_cast<int>(this->targetPoses_.size())) {
    this->targetPose_ = this->targetPoses_.at(this->idx_);
  }
}

void GazeboRosActorCommand::ConfigureRosPublishers(
  const std::shared_ptr<const sdf::Element> &_sdf)
{
  if (_sdf->HasElement("publish_human_state"))
  {
    this->publishHumanState_ = _sdf->Get<bool>("publish_human_state");
  }
  if (!this->publishHumanState_)
  {
    return;
  }

  if (_sdf->HasElement("human_pose_topic"))
  {
    this->humanPoseTopic_ = _sdf->Get<std::string>("human_pose_topic");
  }
  if (_sdf->HasElement("human_path_topic"))
  {
    this->humanPathTopic_ = _sdf->Get<std::string>("human_path_topic");
  }
  if (_sdf->HasElement("human_frame_id"))
  {
    this->humanFrameId_ = _sdf->Get<std::string>("human_frame_id");
  }
  if (_sdf->HasElement("human_publish_rate"))
  {
    const double publishRate = _sdf->Get<double>("human_publish_rate");
    if (publishRate > 0.0)
    {
      this->humanPublishPeriod_ =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / publishRate));
    }
  }

  auto context = rclcpp::contexts::get_global_default_context();
  if (!context->is_valid())
  {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  this->rosNode_ = std::make_shared<rclcpp::Node>(
    "gazebo_actor_state_publisher");
  this->humanPosePub_ =
    this->rosNode_->create_publisher<geometry_msgs::msg::PoseStamped>(
      this->humanPoseTopic_, 10);

  auto pathQos = rclcpp::QoS(1).transient_local();
  this->humanPathPub_ =
    this->rosNode_->create_publisher<nav_msgs::msg::Path>(
      this->humanPathTopic_, pathQos);
}

void GazeboRosActorCommand::PublishHumanPose(
  const gz::math::Pose3d &_pose,
  const std::chrono::steady_clock::duration &_simTime)
{
  if (!this->publishHumanState_ || !this->humanPosePub_)
  {
    return;
  }

  if (this->lastHumanPublish_ != std::chrono::steady_clock::duration::zero() &&
      _simTime - this->lastHumanPublish_ < this->humanPublishPeriod_)
  {
    return;
  }

  this->lastHumanPublish_ = _simTime;

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = ToRosTime(_simTime);
  msg.header.frame_id = this->humanFrameId_;
  msg.pose.position.x = _pose.Pos().X();
  msg.pose.position.y = _pose.Pos().Y();
  msg.pose.position.z = _pose.Pos().Z();
  msg.pose.orientation.x = _pose.Rot().X();
  msg.pose.orientation.y = _pose.Rot().Y();
  msg.pose.orientation.z = _pose.Rot().Z();
  msg.pose.orientation.w = _pose.Rot().W();
  this->humanPosePub_->publish(msg);
  this->PublishHumanPath(_simTime);
}

void GazeboRosActorCommand::PublishHumanPath(
  const std::chrono::steady_clock::duration &_simTime)
{
  if (!this->publishHumanState_ || !this->humanPathPub_ ||
      this->targetPoses_.empty())
  {
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = ToRosTime(_simTime);
  path.header.frame_id = this->humanFrameId_;

  const auto poseCount = this->targetPoses_.size();
  constexpr double kMinHeadingSegmentLength = 1e-6;

  for (std::size_t poseIdx = 0; poseIdx < poseCount; ++poseIdx)
  {
    const auto &target = this->targetPoses_.at(poseIdx);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = target.X();
    pose.pose.position.y = target.Y();

    auto terrainZ = this->TerrainHeight(target.X(), target.Y());
    pose.pose.position.z = terrainZ.has_value() ?
      terrainZ.value() + this->terrainZOffset_ : this->fallbackZ_;

    double yaw = target.Z();
    if (poseCount > 1)
    {
      gz::math::Vector2d segment;
      if (poseIdx + 1 < poseCount)
      {
        const auto &nextTarget = this->targetPoses_.at(poseIdx + 1);
        segment.X(nextTarget.X() - target.X());
        segment.Y(nextTarget.Y() - target.Y());
      }
      else
      {
        const auto &previousTarget = this->targetPoses_.at(poseIdx - 1);
        segment.X(target.X() - previousTarget.X());
        segment.Y(target.Y() - previousTarget.Y());
      }

      if (segment.Length() > kMinHeadingSegmentLength)
      {
        yaw = std::atan2(segment.Y(), segment.X());
      }
    }

    gz::math::Quaterniond quat(0.0, 0.0, yaw);
    pose.pose.orientation.x = quat.X();
    pose.pose.orientation.y = quat.Y();
    pose.pose.orientation.z = quat.Z();
    pose.pose.orientation.w = quat.W();
    path.poses.push_back(pose);
  }

  this->humanPathPub_->publish(path);
}

void GazeboRosActorCommand::LoadTerrainMeshes(const sdf::ElementPtr &_terrainElem)
{
  if (!_terrainElem || !_terrainElem->HasElement("mesh"))
  {
    ignwarn << "Ignoring <terrain> entry without a <mesh> child." << std::endl;
    return;
  }

  TerrainMesh terrain;
  terrain.uri = _terrainElem->GetElement("mesh")->Get<std::string>();
  if (_terrainElem->HasElement("pose"))
  {
    terrain.pose = _terrainElem->Get<gz::math::Pose3d>("pose");
  }

  if (this->LoadTerrainMesh(terrain))
  {
    this->terrainMeshes_.push_back(terrain);
    this->terrainLoaded_ = true;
  }
}

bool GazeboRosActorCommand::LoadTerrainMesh(const TerrainMesh &_terrain)
{
  const auto *mesh = gz::common::MeshManager::Instance()->Load(_terrain.uri);
  if (!mesh)
  {
    ignwarn << "Failed to load terrain mesh [" << _terrain.uri << "]" << std::endl;
    return false;
  }

  std::size_t triangleCount = 0;
  for (unsigned int subMeshIdx = 0; subMeshIdx < mesh->SubMeshCount(); ++subMeshIdx)
  {
    auto subMeshWeak = mesh->SubMeshByIndex(subMeshIdx);
    auto subMesh = subMeshWeak.lock();
    if (!subMesh ||
        subMesh->SubMeshPrimitiveType() != gz::common::SubMesh::TRIANGLES)
    {
      continue;
    }

    for (unsigned int i = 0; i + 2 < subMesh->IndexCount(); i += 3)
    {
      const int aIdx = subMesh->Index(i);
      const int bIdx = subMesh->Index(i + 1);
      const int cIdx = subMesh->Index(i + 2);
      if (aIdx < 0 || bIdx < 0 || cIdx < 0)
      {
        continue;
      }

      TerrainTriangle triangle;
      triangle.a = TransformPoint(_terrain.pose, subMesh->Vertex(aIdx));
      triangle.b = TransformPoint(_terrain.pose, subMesh->Vertex(bIdx));
      triangle.c = TransformPoint(_terrain.pose, subMesh->Vertex(cIdx));
      triangle.minX = std::min({triangle.a.X(), triangle.b.X(), triangle.c.X()});
      triangle.maxX = std::max({triangle.a.X(), triangle.b.X(), triangle.c.X()});
      triangle.minY = std::min({triangle.a.Y(), triangle.b.Y(), triangle.c.Y()});
      triangle.maxY = std::max({triangle.a.Y(), triangle.b.Y(), triangle.c.Y()});

      this->terrainTriangles_.push_back(triangle);
      ++triangleCount;
    }
  }

  ignmsg << "Loaded terrain mesh [" << _terrain.uri << "] with "
         << triangleCount << " triangles" << std::endl;
  return triangleCount > 0;
}

std::optional<double> GazeboRosActorCommand::TerrainHeight(double _x, double _y) const
{
  std::optional<double> bestZ;
  for (const auto &triangle : this->terrainTriangles_)
  {
    if (_x < triangle.minX || _x > triangle.maxX ||
        _y < triangle.minY || _y > triangle.maxY)
    {
      continue;
    }

    const auto z = IntersectVerticalRay(triangle, _x, _y);
    if (!z.has_value())
    {
      continue;
    }

    if (!bestZ.has_value() || z.value() > bestZ.value())
    {
      bestZ = z.value();
    }
  }

  return bestZ;
}

gz::math::Vector3d GazeboRosActorCommand::TransformPoint(
  const gz::math::Pose3d &_pose,
  const gz::math::Vector3d &_point)
{
  return _pose.Pos() + _pose.Rot() * _point;
}

std::optional<double> GazeboRosActorCommand::IntersectVerticalRay(
  const TerrainTriangle &_triangle,
  double _x,
  double _y)
{
  const gz::math::Vector2d a(_triangle.a.X(), _triangle.a.Y());
  const gz::math::Vector2d b(_triangle.b.X(), _triangle.b.Y());
  const gz::math::Vector2d c(_triangle.c.X(), _triangle.c.Y());
  const gz::math::Vector2d p(_x, _y);

  const gz::math::Vector2d v0 = b - a;
  const gz::math::Vector2d v1 = c - a;
  const gz::math::Vector2d v2 = p - a;
  const double denom = v0.X() * v1.Y() - v1.X() * v0.Y();
  constexpr double eps = 1e-9;

  if (std::abs(denom) < eps)
  {
    return std::nullopt;
  }

  const double invDenom = 1.0 / denom;
  const double u = (v2.X() * v1.Y() - v1.X() * v2.Y()) * invDenom;
  const double v = (v0.X() * v2.Y() - v2.X() * v0.Y()) * invDenom;
  const double w = 1.0 - u - v;

  if (u < -eps || v < -eps || w < -eps)
  {
    return std::nullopt;
  }

  return u * _triangle.b.Z() + v * _triangle.c.Z() + w * _triangle.a.Z();
}

builtin_interfaces::msg::Time GazeboRosActorCommand::ToRosTime(
  const std::chrono::steady_clock::duration &_simTime)
{
  const auto seconds =
    std::chrono::duration_cast<std::chrono::seconds>(_simTime);
  const auto nanoseconds =
    std::chrono::duration_cast<std::chrono::nanoseconds>(_simTime - seconds);

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(seconds.count());
  stamp.nanosec = static_cast<uint32_t>(nanoseconds.count());
  return stamp;
}

IGNITION_ADD_PLUGIN(
  gazebo_ros_actor_plugin::GazeboRosActorCommand,
  gz::sim::System,
  GazeboRosActorCommand::ISystemConfigure,
  GazeboRosActorCommand::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  gazebo_ros_actor_plugin::GazeboRosActorCommand,
  "gazebo_ros_actor_plugin::GazeboRosActorCommand")
