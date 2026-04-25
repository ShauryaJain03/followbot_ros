#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sdf/Element.hh>

namespace gazebo_ros_actor_plugin {

class GazeboRosActorCommand :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
 public:
   GazeboRosActorCommand();
   void Configure(const gz::sim::Entity &_entity,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  gz::sim::EntityComponentManager &_ecm,
                  gz::sim::EventManager &_eventMgr) override;
   void PreUpdate(const gz::sim::UpdateInfo &_info,
                  gz::sim::EntityComponentManager &_ecm) override;

 private:
   struct TerrainMesh
   {
     std::string uri;
     gz::math::Pose3d pose = gz::math::Pose3d::Zero;
   };

   struct TerrainTriangle
   {
     gz::math::Vector3d a;
     gz::math::Vector3d b;
     gz::math::Vector3d c;
     double minX = 0.0;
     double maxX = 0.0;
     double minY = 0.0;
     double maxY = 0.0;
   };

   void VelCallback(const gz::msgs::Twist &msg);
   void PathCallback(const gz::msgs::Pose_V &msg);
   void ChooseNewTarget();
   void ConfigureRosPublishers(const std::shared_ptr<const sdf::Element> &_sdf);
   void PublishHumanPose(
     const gz::math::Pose3d &_pose,
     const std::chrono::steady_clock::duration &_simTime);
   void PublishHumanPath(
     const std::chrono::steady_clock::duration &_simTime);
   void LoadTerrainMeshes(const sdf::ElementPtr &_terrainElem);
   bool LoadTerrainMesh(const TerrainMesh &_terrain);
   std::optional<double> TerrainHeight(double _x, double _y) const;
   static gz::math::Vector3d TransformPoint(
     const gz::math::Pose3d &_pose,
     const gz::math::Vector3d &_point);
   static std::optional<double> IntersectVerticalRay(
     const TerrainTriangle &_triangle,
     double _x,
     double _y);
   static builtin_interfaces::msg::Time ToRosTime(
     const std::chrono::steady_clock::duration &_simTime);

   gz::transport::Node node_;
   rclcpp::Node::SharedPtr rosNode_;
   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr humanPosePub_;
   rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr humanPathPub_;
   std::string velTopic_;
   std::string pathTopic_;
   std::string humanPoseTopic_;
   std::string humanPathTopic_;
   std::string humanFrameId_;
   gz::sim::Entity actorEntity_;
   double animationFactor_;
   std::chrono::steady_clock::duration lastUpdate_;
   std::chrono::steady_clock::duration lastHumanPublish_;
   std::chrono::steady_clock::duration humanPublishPeriod_;
   std::string followMode_;
   gz::math::Pose3d targetVel_;
   double linVelocity_;
   double angVelocity_;
   gz::math::Vector3d targetPose_;
   std::vector<gz::math::Vector3d> targetPoses_;
   int idx_;
   double linTolerance_;
   double angTolerance_;
   double defaultRotation_;
   std::queue<gz::math::Vector3d> cmdQueue_;
   std::queue<std::vector<gz::math::Vector3d>> pathQueue_;
   std::mutex mutex_;
   bool pathCompletedLogged_;
   bool publishHumanState_;
   bool humanPathDirty_;
   std::vector<TerrainMesh> terrainMeshes_;
   std::vector<TerrainTriangle> terrainTriangles_;
   bool terrainLoaded_;
   double terrainZOffset_;
   double fallbackZ_;
};

} // namespace gazebo_ros_actor_plugin
