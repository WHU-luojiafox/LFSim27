// Copyright 2026 RMOSS contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef RMOSS_OMNI_DRIVE_HH
#define RMOSS_OMNI_DRIVE_HH
#include <memory>
#include <ignition/gazebo/System.hh>

namespace rmoss
{
class OmniDrivePrivate;
class OmniDrive : public ignition::gazebo::System,
                  public ignition::gazebo::ISystemConfigure,
                  public ignition::gazebo::ISystemPreUpdate,
                  public ignition::gazebo::ISystemPostUpdate
{
public:
  OmniDrive();
  ~OmniDrive() override;
  void Configure(const ignition::gazebo::Entity &,
    const std::shared_ptr<const sdf::Element> &,
    ignition::gazebo::EntityComponentManager &, ignition::gazebo::EventManager &) override;
  void PreUpdate(const ignition::gazebo::UpdateInfo &,
    ignition::gazebo::EntityComponentManager &) override;
  void PostUpdate(const ignition::gazebo::UpdateInfo &,
    const ignition::gazebo::EntityComponentManager &) override;
private:
  std::unique_ptr<OmniDrivePrivate> data;
};
}
#endif
