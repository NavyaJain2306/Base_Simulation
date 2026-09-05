// Copyright 2026 Jacob
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LINK_ATTATCHER__LINK_ATTATCHER_HH_
#define LINK_ATTATCHER__LINK_ATTATCHER_HH_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>
#include <rclcpp/rclcpp.hpp>
#include <sdf/Element.hh>

#include "link_attatcher/srv/attach.hpp"

namespace link_attatcher
{

struct AttachRequest
{
    std::string model1;
    std::string link1;

    std::string model2;
    std::string link2;

    bool attach;
};


class LinkAttatcherPlugin :
    public ignition::gazebo::System,
    public ignition::gazebo::ISystemConfigure,
    public ignition::gazebo::ISystemPreUpdate
{
public:

    LinkAttatcherPlugin() = default;

    ~LinkAttatcherPlugin() override = default;


    // Called once when Gazebo loads the plugin
    void Configure(
        const ignition::gazebo::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        ignition::gazebo::EntityComponentManager &_ecm,
        ignition::gazebo::EventManager &_eventMgr) override;


    // Called every simulation iteration
    void PreUpdate(
        const ignition::gazebo::UpdateInfo &_info,
        ignition::gazebo::EntityComponentManager &_ecm) override;


private:

    // Attach two links using a fixed joint
    bool AttachJoint(
        ignition::gazebo::EntityComponentManager &_ecm,
        const std::string &model1,
        const std::string &link1,
        const std::string &model2,
        const std::string &link2);


    // Remove an existing joint
    bool DetachJoint(
        ignition::gazebo::EntityComponentManager &_ecm,
        const std::string &model1,
        const std::string &link1,
        const std::string &model2,
        const std::string &link2);


    // Find a link entity from model + link name
    ignition::gazebo::Entity FindLink(
        ignition::gazebo::EntityComponentManager &_ecm,
        const std::string &model,
        const std::string &link);


private:

    // ROS 2 node used by the Gazebo plugin
    std::shared_ptr<rclcpp::Node> rosNode;


    // ROS 2 attach service
    rclcpp::Service<link_attatcher::srv::Attach>::SharedPtr attachSrv;


    // ROS 2 detach service
    rclcpp::Service<link_attatcher::srv::Attach>::SharedPtr detachSrv;


    // Protects pendingRequests and activeJoints
    std::mutex mutex;


    // Requests received from ROS 2 services
    std::vector<AttachRequest> pendingRequests;


    // Information about joints currently created
    struct ActiveJoint
    {
        std::string model1;
        std::string link1;

        std::string model2;
        std::string link2;

        ignition::gazebo::Entity jointEntity;
    };


    // Currently active detachable joints
    std::vector<ActiveJoint> activeJoints;
};

}  // namespace link_attatcher

#endif  // LINK_ATTATCHER__LINK_ATTATCHER_HH_