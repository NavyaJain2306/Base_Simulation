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

#include "link_attatcher/link_attatcher.hh"

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/DetachableJoint.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/plugin/Register.hh>

namespace link_attatcher
{

//////////////////////////////////////////////////////////////
// Configure
//////////////////////////////////////////////////////////////

void LinkAttatcherPlugin::Configure(
    const ignition::gazebo::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    ignition::gazebo::EntityComponentManager &_ecm,
    ignition::gazebo::EventManager &_eventMgr)
{
    (void)_entity;
    (void)_sdf;
    (void)_ecm;
    (void)_eventMgr;

    // Initialize ROS 2
    if (!rclcpp::ok())
    {
        int argc = 0;
        char **argv = nullptr;
        rclcpp::init(argc, argv);
    }

    this->rosNode =
        std::make_shared<rclcpp::Node>("link_attatcher");

    //////////////////////////////////////////////////////////
    // ATTACH SERVICE
    //////////////////////////////////////////////////////////

    this->attachSrv =
        this->rosNode->create_service<link_attatcher::srv::Attach>(
            "/link_attacher/attach",
            [this](
                const std::shared_ptr<link_attatcher::srv::Attach::Request> request,
                std::shared_ptr<link_attatcher::srv::Attach::Response> response)
            {
                std::lock_guard<std::mutex> lock(this->mutex);

                AttachRequest req;

                req.model1 = request->model1_name;
                req.link1  = request->link1_name;

                req.model2 = request->model2_name;
                req.link2  = request->link2_name;

                req.attach = true;

                this->pendingRequests.push_back(req);

                response->success = true;
                response->message = "Attach request received";
            });

    //////////////////////////////////////////////////////////
    // DETACH SERVICE
    //////////////////////////////////////////////////////////

    this->detachSrv =
        this->rosNode->create_service<link_attatcher::srv::Attach>(
            "/link_attacher/detach",
            [this](
                const std::shared_ptr<link_attatcher::srv::Attach::Request> request,
                std::shared_ptr<link_attatcher::srv::Attach::Response> response)
            {
                std::lock_guard<std::mutex> lock(this->mutex);

                AttachRequest req;

                req.model1 = request->model1_name;
                req.link1  = request->link1_name;

                req.model2 = request->model2_name;
                req.link2  = request->link2_name;

                req.attach = false;

                this->pendingRequests.push_back(req);

                response->success = true;
                response->message = "Detach request received";
            });

    ignmsg << "LinkAttatcherPlugin configured.\n";
    ignmsg << "Attach service: /link_attacher/attach\n";
    ignmsg << "Detach service: /link_attacher/detach\n";
}


//////////////////////////////////////////////////////////////
// PreUpdate
//////////////////////////////////////////////////////////////

void LinkAttatcherPlugin::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
    (void)_info;

    // Process ROS callbacks
    if (this->rosNode)
    {
        rclcpp::spin_some(this->rosNode);
    }

    //////////////////////////////////////////////////////////
    // Get pending requests
    //////////////////////////////////////////////////////////

    std::vector<AttachRequest> requests;

    {
        std::lock_guard<std::mutex> lock(this->mutex);

        requests.swap(this->pendingRequests);
    }

    //////////////////////////////////////////////////////////
    // Process requests
    //////////////////////////////////////////////////////////

    for (const auto &request : requests)
    {
        if (request.attach)
        {
            this->AttachJoint(
                _ecm,
                request.model1,
                request.link1,
                request.model2,
                request.link2);
        }
        else
        {
            this->DetachJoint(
                _ecm,
                request.model1,
                request.link1,
                request.model2,
                request.link2);
        }
    }
}


//////////////////////////////////////////////////////////////
// FindLink
//////////////////////////////////////////////////////////////

ignition::gazebo::Entity LinkAttatcherPlugin::FindLink(
    ignition::gazebo::EntityComponentManager &_ecm,
    const std::string &_model,
    const std::string &_link)
{
    ignition::gazebo::Entity modelEntity =
        ignition::gazebo::kNullEntity;

    //////////////////////////////////////////////////////////
    // Find model
    //////////////////////////////////////////////////////////

    _ecm.Each<ignition::gazebo::components::Name>(
        [&](const ignition::gazebo::Entity &_entity,
            const ignition::gazebo::components::Name *_name)
        {
            if (_name->Data() == _model)
            {
                modelEntity = _entity;
                return false;
            }

            return true;
        });

    if (modelEntity == ignition::gazebo::kNullEntity)
    {
        ignerr << "Could not find model: "
               << _model << "\n";

        return ignition::gazebo::kNullEntity;
    }

    //////////////////////////////////////////////////////////
    // Find link
    //////////////////////////////////////////////////////////

    ignition::gazebo::Entity linkEntity =
        ignition::gazebo::kNullEntity;

    _ecm.Each<ignition::gazebo::components::Name>(
        [&](const ignition::gazebo::Entity &_entity,
            const ignition::gazebo::components::Name *_name)
        {
            if (_name->Data() == _link)
            {
                // Make sure the link belongs to the requested
                // model by checking its parent.

                auto parent =
                    _ecm.ParentEntity(_entity);

                if (parent == modelEntity)
                {
                    linkEntity = _entity;
                    return false;
                }
            }

            return true;
        });

    if (linkEntity == ignition::gazebo::kNullEntity)
    {
        ignerr << "Could not find link: "
               << _model << "/" << _link << "\n";
    }

    return linkEntity;
}


//////////////////////////////////////////////////////////////
// AttachJoint
//////////////////////////////////////////////////////////////

bool LinkAttatcherPlugin::AttachJoint(
    ignition::gazebo::EntityComponentManager &_ecm,
    const std::string &_model1,
    const std::string &_link1,
    const std::string &_model2,
    const std::string &_link2)
{
    //////////////////////////////////////////////////////////
    // Find links
    //////////////////////////////////////////////////////////

    auto link1Entity =
        this->FindLink(
            _ecm,
            _model1,
            _link1);

    auto link2Entity =
        this->FindLink(
            _ecm,
            _model2,
            _link2);

    if (link1Entity == ignition::gazebo::kNullEntity ||
        link2Entity == ignition::gazebo::kNullEntity)
    {
        ignerr << "Could not find requested links to attach.\n";

        return false;
    }

    //////////////////////////////////////////////////////////
    // Check if already attached
    //////////////////////////////////////////////////////////

    for (const auto &joint : this->activeJoints)
    {
        if ((joint.model1 == _model1 &&
             joint.link1 == _link1 &&
             joint.model2 == _model2 &&
             joint.link2 == _link2) ||

            (joint.model1 == _model2 &&
             joint.link1 == _link2 &&
             joint.model2 == _model1 &&
             joint.link2 == _link1))
        {
            ignmsg << "Links are already attached.\n";

            return false;
        }
    }

    //////////////////////////////////////////////////////////
    // Create joint entity
    //////////////////////////////////////////////////////////

    auto jointEntity = _ecm.CreateEntity();

    _ecm.CreateComponent(
        jointEntity,
        ignition::gazebo::components::Name(
            "runtime_fixed_joint_" +
            std::to_string(jointEntity)));

    //////////////////////////////////////////////////////////
    // Create DetachableJoint component
    //////////////////////////////////////////////////////////

    ignition::gazebo::components::DetachableJointInfo jointInfo;

    jointInfo.parentLink = link1Entity;
    jointInfo.childLink  = link2Entity;
    jointInfo.jointType  = "fixed";

    _ecm.CreateComponent(
        jointEntity,
        ignition::gazebo::components::DetachableJoint(
            jointInfo));

    //////////////////////////////////////////////////////////
    // Save joint
    //////////////////////////////////////////////////////////

    {
        std::lock_guard<std::mutex> lock(this->mutex);

        this->activeJoints.push_back(
            {
                _model1,
                _link1,
                _model2,
                _link2,
                jointEntity
            });
    }

    ignmsg
        << "Attached "
        << _model1 << "/" << _link1
        << " to "
        << _model2 << "/" << _link2
        << "\n";

    return true;
}


//////////////////////////////////////////////////////////////
// DetachJoint
//////////////////////////////////////////////////////////////

bool LinkAttatcherPlugin::DetachJoint(
    ignition::gazebo::EntityComponentManager &_ecm,
    const std::string &_model1,
    const std::string &_link1,
    const std::string &_model2,
    const std::string &_link2)
{
    std::lock_guard<std::mutex> lock(this->mutex);

    //////////////////////////////////////////////////////////
    // Find active joint
    //////////////////////////////////////////////////////////

    auto it = std::find_if(
        this->activeJoints.begin(),
        this->activeJoints.end(),
        [&](const ActiveJoint &joint)
        {
            return
                (joint.model1 == _model1 &&
                 joint.link1 == _link1 &&
                 joint.model2 == _model2 &&
                 joint.link2 == _link2)

                ||

                (joint.model1 == _model2 &&
                 joint.link1 == _link2 &&
                 joint.model2 == _model1 &&
                 joint.link2 == _link1);
        });

    if (it == this->activeJoints.end())
    {
        ignerr
            << "No active joint found between "
            << _model1 << "/" << _link1
            << " and "
            << _model2 << "/" << _link2
            << "\n";

        return false;
    }

    //////////////////////////////////////////////////////////
    // Remove joint entity
    //////////////////////////////////////////////////////////

    if (it->jointEntity != ignition::gazebo::kNullEntity)
    {
        _ecm.RequestRemoveEntity(
            it->jointEntity);
    }

    ignmsg
        << "Detached "
        << _model1 << "/" << _link1
        << " from "
        << _model2 << "/" << _link2
        << "\n";

    //////////////////////////////////////////////////////////
    // Remove from active list
    //////////////////////////////////////////////////////////

    this->activeJoints.erase(it);

    return true;
}

}  // namespace link_attatcher

//////////////////////////////////////////////////////////////
// Gazebo plugin registration
//////////////////////////////////////////////////////////////

IGNITION_ADD_PLUGIN(
    link_attatcher::LinkAttatcherPlugin,
    ignition::gazebo::System,
    ignition::gazebo::ISystemConfigure,
    ignition::gazebo::ISystemPreUpdate
)