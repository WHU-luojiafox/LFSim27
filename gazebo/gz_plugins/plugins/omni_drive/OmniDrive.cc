// Copyright 2026 RMOSS contributors
// SPDX-License-Identifier: Apache-2.0
#include "OmniDrive.hh"
#include "OmniKinematics.hh"
#include <chrono>
#include <mutex>
#include <set>
#include <ignition/common/Console.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

using namespace ignition;
using namespace ignition::gazebo;

namespace rmoss
{
class OmniDrivePrivate
{
public:
  void Command(const msgs::Twist &msg)
  {
    std::lock_guard<std::mutex> lock(mutex);
    command = Eigen::Vector3d(msg.linear().x(), msg.linear().y(), msg.angular().z());
    if (!command.allFinite()) command.setZero();
    lastCommand = simTime;
    haveCommand = true;
  }
  bool ready{false}, haveCommand{false};
  Entity chassis{kNullEntity};
  std::vector<Entity> joints;
  std::unique_ptr<OmniKinematics> kinematics;
  transport::Node node;
  transport::Node::Publisher odom;
  std::string odomFrame, baseFrame, commandTopic;
  std::mutex mutex;
  Eigen::Vector3d command{Eigen::Vector3d::Zero()};
  double simTime{0}, lastCommand{0}, lastOdom{-1};
  double timeout{0.5}, odomFrequency{50};
  double maxSpeed{2}, maxYawRate{3};
  double linearGain{120}, angularGain{60}, maxForce{150}, maxTorque{50};
};

OmniDrive::OmniDrive() : data(std::make_unique<OmniDrivePrivate>()) {}
OmniDrive::~OmniDrive()
{
  // Stop transport callbacks before destroying command state and its mutex.
  if (!data->commandTopic.empty()) data->node.Unsubscribe(data->commandTopic);
}

void OmniDrive::Configure(const Entity &entity,
  const std::shared_ptr<const sdf::Element> &sdf,
  EntityComponentManager &ecm, EventManager &)
{
  try
  {
    Model model(entity);
    if (!model.Valid(ecm)) throw std::invalid_argument("attach OmniDrive to a model");
    auto config = sdf->Clone();
    auto required = [](const sdf::ElementPtr &element, const std::string &key) {
      if (!element->HasElement(key)) throw std::invalid_argument("missing element: " + key);
    };
    required(config, "chassis_link");
    const auto linkName = config->Get<std::string>("chassis_link");
    data->chassis = model.LinkByName(ecm, linkName);
    if (data->chassis == kNullEntity) throw std::invalid_argument("chassis link not found");
    required(config, "wheel");
    std::vector<OmniWheel> wheels;
    std::set<std::string> names;
    for (auto wheel = config->GetElement("wheel"); wheel; wheel = wheel->GetNextElement("wheel"))
    {
      for (const auto &key : {"joint", "x", "y", "rolling_angle", "radius", "max_angular_speed"})
        required(wheel, key);
      const auto name = wheel->Get<std::string>("joint");
      const Entity joint = model.JointByName(ecm, name);
      if (joint == kNullEntity || !names.insert(name).second)
        throw std::invalid_argument("missing or duplicate wheel joint: " + name);
      data->joints.push_back(joint);
      wheels.push_back({wheel->Get<double>("x"), wheel->Get<double>("y"),
        wheel->Get<double>("rolling_angle"), wheel->Get<double>("radius"),
        wheel->Get<double>("max_angular_speed")});
    }
    data->kinematics = std::make_unique<OmniKinematics>(wheels);
    auto positive = [&](const std::string &key, double &value) {
      value = config->Get<double>(key, value).first;
      if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("invalid " + key);
    };
    positive("command_timeout", data->timeout);
    positive("odom_frequency", data->odomFrequency);
    positive("max_linear_speed", data->maxSpeed);
    positive("max_yaw_rate", data->maxYawRate);
    positive("linear_gain", data->linearGain);
    positive("angular_gain", data->angularGain);
    positive("max_force", data->maxForce);
    positive("max_torque", data->maxTorque);
    const auto prefix = "/model/" + model.Name(ecm);
    const auto topic = config->Get<std::string>("topic", prefix + "/cmd_vel").first;
    data->commandTopic = topic;
    const auto odomTopic = config->Get<std::string>("odom_topic", prefix + "/odometry").first;
    data->odomFrame = config->Get<std::string>("odom_frame", "odom").first;
    data->baseFrame = config->Get<std::string>("base_frame", linkName).first;
    data->odom = data->node.Advertise<msgs::Odometry>(odomTopic);
    if (!data->odom || !data->node.Subscribe(topic, &OmniDrivePrivate::Command, data.get()))
      throw std::invalid_argument("invalid transport topic");
    Link(data->chassis).EnableVelocityChecks(ecm);
    data->ready = true;
    ignmsg << "OmniDrive: " << wheels.size() << " wheels, command " << topic << std::endl;
  }
  catch (const std::exception &error)
  {
    ignerr << "OmniDrive disabled: " << error.what() << std::endl;
  }
}

void OmniDrive::PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm)
{
  if (!data->ready) return;
  Eigen::Vector3d target = Eigen::Vector3d::Zero();
  const double now = std::chrono::duration<double>(info.simTime).count();
  {
    std::lock_guard<std::mutex> lock(data->mutex);
    if (now < data->simTime)
    {
      data->haveCommand = false;
      data->command.setZero();
      data->lastOdom = -1;
    }
    data->simTime = now;
    if (data->haveCommand && now - data->lastCommand <= data->timeout)
      target = data->command;
  }
  if (info.paused || info.dt <= std::chrono::steady_clock::duration::zero()) return;
  // Preserve the ratios of vx, vy and omega for all actuator limits.
  const double scale = std::max({1.0, std::hypot(target.x(), target.y()) / data->maxSpeed,
    std::abs(target.z()) / data->maxYawRate});
  target = data->kinematics->limit(target / scale);
  const auto wheelSpeeds = data->kinematics->wheelSpeeds(target);
  for (size_t i = 0; i < data->joints.size(); ++i)
    ecm.SetComponentData<components::JointVelocityCmd>(data->joints[i], {wheelSpeeds[i]});

  const Link link(data->chassis);
  const auto pose = link.WorldPose(ecm);
  const auto worldLinear = link.WorldLinearVelocity(ecm);
  const auto worldAngular = link.WorldAngularVelocity(ecm);
  if (!pose || !worldLinear || !worldAngular) return;
  const auto linear = pose->Rot().RotateVectorReverse(*worldLinear);
  const auto angular = pose->Rot().RotateVectorReverse(*worldAngular);
  math::Vector3d force(data->linearGain * (target.x() - linear.X()),
    data->linearGain * (target.y() - linear.Y()), 0);
  if (force.Length() > data->maxForce) force *= data->maxForce / force.Length();
  const double torque = std::max(-data->maxTorque, std::min(data->maxTorque,
    data->angularGain * (target.z() - angular.Z())));
  // Equivalent planar traction, as in MecanumDrive2; collisions remain physical.
  link.AddWorldWrench(ecm, pose->Rot().RotateVector(force),
    pose->Rot().RotateVector(math::Vector3d(0, 0, torque)));
}

void OmniDrive::PostUpdate(const UpdateInfo &info, const EntityComponentManager &ecm)
{
  if (!data->ready || info.paused) return;
  const double now = std::chrono::duration<double>(info.simTime).count();
  if (now - data->lastOdom < 1.0 / data->odomFrequency) return;
  const Link link(data->chassis);
  const auto pose = link.WorldPose(ecm);
  const auto linear = link.WorldLinearVelocity(ecm);
  const auto angular = link.WorldAngularVelocity(ecm);
  if (!pose || !linear || !angular) return;
  msgs::Odometry msg;
  // Ground truth in world-aligned odom; twist is in the child/body frame.
  msgs::Set(msg.mutable_pose(), *pose);
  msgs::Set(msg.mutable_twist()->mutable_linear(), pose->Rot().RotateVectorReverse(*linear));
  msgs::Set(msg.mutable_twist()->mutable_angular(), pose->Rot().RotateVectorReverse(*angular));
  msg.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(info.simTime));
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value(data->odomFrame);
  auto child = msg.mutable_header()->add_data();
  child->set_key("child_frame_id");
  child->add_value(data->baseFrame);
  data->odom.Publish(msg);
  data->lastOdom = now;
}
}  // namespace rmoss

IGNITION_ADD_PLUGIN(rmoss::OmniDrive, ignition::gazebo::System,
  rmoss::OmniDrive::ISystemConfigure, rmoss::OmniDrive::ISystemPreUpdate,
  rmoss::OmniDrive::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(rmoss::OmniDrive, "rmoss::OmniDrive")
