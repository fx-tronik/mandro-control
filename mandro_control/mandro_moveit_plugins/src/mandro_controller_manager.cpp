/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Fraunhofer IPA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Fraunhofer IPA nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mathias Lüdtke */

#include <ros/ros.h>

#include <moveit/macros/class_forward.h>

#include <moveit_ros_control_interface/ControllerHandle.h>

#include <moveit/controller_manager/controller_manager.h>

#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>

#include <pluginlib/class_list_macros.hpp>
#include <pluginlib/class_loader.hpp>

#include <boost/bimap.hpp>

#include <map>
#include <memory>

namespace moveit_ros_control_interface
{
/**
 * \brief check for timeout
 * @param t timestamp to check, is update if timeout duration was passed
 * @param[in] timeout timeout duration in seconds
 * @param[in] force force timeout
 * @return True if timeout duration was passed
 */
bool checkTimeout(ros::Time& t, double timeout, bool force = false)
{
  ros::Time now = ros::Time::now();
  if (force || (now - t) >= ros::Duration(timeout))
  {
    t = now;
    return true;
  }
  return false;
}

MOVEIT_CLASS_FORWARD(MandroMoveItControllerManager);

/**
 * \brief moveit_controller_manager::MoveItControllerManager sub class that interfaces one ros_control
 * controller_manager
 * instance.
 * All services and names are relative to ns_.
 */
class MandroMoveItControllerManager : public moveit_controller_manager::MoveItControllerManager
{
  const std::string ns_;
  pluginlib::ClassLoader<ControllerHandleAllocator> loader_;
  typedef std::map<std::string, controller_manager_msgs::ControllerState> ControllersMap;
  ControllersMap managed_controllers_;
  ControllersMap active_controllers_;
  typedef std::map<std::string, ControllerHandleAllocatorPtr> AllocatorsMap;
  AllocatorsMap allocators_;

  typedef std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr> HandleMap;
  HandleMap handles_;

  ros::Time controllers_stamp_;
  boost::mutex controllers_mutex_;

  /**
   * \brief Check if given controller is active
   * @param s state of controller
   * @return true if controller is active
   */
  static bool isActive(const controller_manager_msgs::ControllerState& s)
  {
    return s.state == std::string("running");
  }

  /**
   * \brief  Call list_controllers and populate managed_controllers_ and active_controllers_. Allocates handles if
   * needed.
   * Throttled down to 1 Hz, controllers_mutex_ must be locked externally
   * @param force force rediscover
   */
  void discover(bool force = false)
  {
    if (!checkTimeout(controllers_stamp_, 1.0, force))
      return;

    controller_manager_msgs::ListControllers srv;
    if (!ros::service::call(getAbsName("controller_manager/list_controllers"), srv))
    {
      ROS_WARN_STREAM("Failed to read controllers from " << ns_ << "controller_manager/list_controllers");
    }
    managed_controllers_.clear();
    active_controllers_.clear();
    for (const controller_manager_msgs::ControllerState& controller : srv.response.controller)
    {
      if (isActive(controller))
      {
        active_controllers_.insert(std::make_pair(controller.name, controller));  // without namespace
      }
      if (loader_.isClassAvailable(controller.type))
      {
        std::string absname = getAbsName(controller.name);
        managed_controllers_.insert(std::make_pair(absname, controller));  // with namespace
        allocate(absname, controller);
      }
    }
  }

  /**
   * \brief Allocates a MoveItControllerHandle instance for the given controller
   * Might create allocator object first.
   * @param name fully qualified name of the controller
   * @param controller controller information
   */
  void allocate(const std::string& name, const controller_manager_msgs::ControllerState& controller)
  {
    if (handles_.find(name) == handles_.end())
    {
      const std::string& type = controller.type;
      AllocatorsMap::iterator alloc_it = allocators_.find(type);
      if (alloc_it == allocators_.end())
      {  // create allocator is needed
        alloc_it = allocators_.insert(std::make_pair(type, loader_.createUniqueInstance(type))).first;
      }

      std::vector<std::string> resources;
#if defined(MOVEIT_ROS_CONTROL_INTERFACE_OLD_ROS_CONTROL)
      resources = controller.resources;
#else
      // Collect claimed resources across different hardware interfaces
      for (const controller_manager_msgs::HardwareInterfaceResources& claimed_resource : controller.claimed_resources)
      {
        for (const std::string& resource : claimed_resource.resources)
        {
          resources.push_back(resource);
        }
      }
#endif

      moveit_controller_manager::MoveItControllerHandlePtr handle =
          alloc_it->second->alloc(name, resources);  // allocate handle
      if (handle)
        handles_.insert(std::make_pair(name, handle));
    }
  }

  /**
   * \brief get fully qualified name
   * @param name name to be resolved to an absolute name
   * @return resolved name
   */
  std::string getAbsName(const std::string& name)
  {
    return ros::names::append(ns_, name);
  }

public:
  /**
   * \brief The default constructor. Reads the namespace from ~ros_control_namespace param and defaults to /
   */
  MandroMoveItControllerManager()
    : ns_(ros::NodeHandle("~").param("ros_control_namespace", std::string("/")))
    , loader_("moveit_ros_control_interface", "moveit_ros_control_interface::ControllerHandleAllocator")
  {
    ROS_INFO_STREAM("Started moveit_ros_control_interface::MoveItControllerManager for namespace " << ns_);
  }

  /**
   * \brief Configure interface with namespace
   * @param ns namespace of ros_control node (without /controller_manager/)
   */
  MandroMoveItControllerManager(const std::string& ns)
    : ns_(ns), loader_("moveit_ros_control_interface", "moveit_ros_control_interface::ControllerHandleAllocator")
  {
  }

  /**
   * \brief Find and return the pre-allocated handle for the given controller.
   * @param name
   * @return
   */
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    boost::mutex::scoped_lock lock(controllers_mutex_);
    HandleMap::iterator it = handles_.find(name);
    if (it != handles_.end())
    {  // controller is is manager by this interface
      return it->second;
    }
    return moveit_controller_manager::MoveItControllerHandlePtr();
  }

  /**
   * \brief Refresh controller list and output all managed controllers
   * @param[out] names list of controllers (with namespace)
   */
  void getControllersList(std::vector<std::string>& names) override
  {
    boost::mutex::scoped_lock lock(controllers_mutex_);
    discover();

    for (std::pair<const std::string, controller_manager_msgs::ControllerState>& managed_controller :
         managed_controllers_)
    {
      names.push_back(managed_controller.first);
    }
  }

  /**
   * \brief Refresh controller list and output all active, managed controllers
   * @param[out] names list of controllers (with namespace)
   */
  void getActiveControllers(std::vector<std::string>& names) override
  {
    boost::mutex::scoped_lock lock(controllers_mutex_);
    discover();

    for (std::pair<const std::string, controller_manager_msgs::ControllerState>& managed_controller :
         managed_controllers_)
    {
      if (isActive(managed_controller.second))
        names.push_back(managed_controller.first);
    }
  }

  /**
   * \brief Read resources from cached controller states
   * @param[in] name name of controller (with namespace)
   * @param[out] joints
   */
  void getControllerJoints(const std::string& name, std::vector<std::string>& joints) override
  {
    boost::mutex::scoped_lock lock(controllers_mutex_);
    ControllersMap::iterator it = managed_controllers_.find(name);
    if (it != managed_controllers_.end())
    {
#if defined(MOVEIT_ROS_CONTROL_INTERFACE_OLD_ROS_CONTROL)
      joints = it->second.resources;
#else
      for (controller_manager_msgs::HardwareInterfaceResources& claimed_resource : it->second.claimed_resources)
      {
        std::vector<std::string>& resources = claimed_resource.resources;
        joints.insert(joints.end(), resources.begin(), resources.end());
      }
#endif
    }
  }

  /**
   * \brief Refresh controller state and output the state of the given one, only active_ will be set
   * @param[in] name name of controller (with namespace)
   * @return state
   */
  ControllerState getControllerState(const std::string& name) override
  {
    boost::mutex::scoped_lock lock(controllers_mutex_);
    discover();

    ControllerState c;
    ControllersMap::iterator it = managed_controllers_.find(name);
    if (it != managed_controllers_.end())
    {
      c.active_ = isActive(it->second);
    }
    return c;
  }

  /**
   * \brief Filter lists for managed controller and computes switching set.
   * Stopped list might be extended by unsupported controllers that claim needed resources
   * @param activate
   * @param deactivate
   * @return true if switching succeeded
   */
  bool switchControllers(const std::vector<std::string>& activate, const std::vector<std::string>& deactivate) override
  {
    boost::mutex::scoped_lock lock(controllers_mutex_);
    discover(true);

    typedef boost::bimap<std::string, std::string> resources_bimap;

    resources_bimap claimed_resources;

    // fill bimap with active controllers and their resources
    for (std::pair<const std::string, controller_manager_msgs::ControllerState>& active_controller :
         active_controllers_)
    {
#if defined(MOVEIT_ROS_CONTROL_INTERFACE_OLD_ROS_CONTROL)
      for (std::vector<std::string>::iterator r = c->second.resources.begin(); r != c->second.resources.end(); ++r)
      {
        claimed_resources.insert(resources_bimap::value_type(c->second.name, *r));
      }
#else
      for (std::vector<controller_manager_msgs::HardwareInterfaceResources>::iterator hir =
               active_controller.second.claimed_resources.begin();
           hir != active_controller.second.claimed_resources.end(); ++hir)
      {
        for (std::string& resource : hir->resources)
        {
          claimed_resources.insert(resources_bimap::value_type(active_controller.second.name, resource));
        }
      }
#endif
    }

    controller_manager_msgs::SwitchController srv;

    for (const std::string& it : deactivate)
    {
      ControllersMap::iterator c = managed_controllers_.find(it);
      if (c != managed_controllers_.end())
      {  // controller belongs to this manager
        srv.request.stop_controllers.push_back(c->second.name);
        claimed_resources.right.erase(c->second.name);  // remove resources
      }
    }

    for (const std::string& it : activate)
    {
      ControllersMap::iterator c = managed_controllers_.find(it);
      if (c != managed_controllers_.end())
      {  // controller belongs to this manager
        srv.request.start_controllers.push_back(c->second.name);
#if defined(MOVEIT_ROS_CONTROL_INTERFACE_OLD_ROS_CONTROL)
        for (std::vector<std::string>::iterator r = c->second.resources.begin(); r != c->second.resources.end(); ++r)
        {  // for all claimed resource
          resources_bimap::right_const_iterator res = claimed_resources.right.find(*r);
          if (res != claimed_resources.right.end())
          {                                                       // resource is claimed
            srv.request.stop_controllers.push_back(res->second);  // add claiming controller to stop list
            claimed_resources.left.erase(res->second);            // remove claimed resources
          }
        }
#else
        for (controller_manager_msgs::HardwareInterfaceResources& claimed_resource : c->second.claimed_resources)
        {
          for (const std::string& resource : claimed_resource.resources)
          {  // for all claimed resource
            resources_bimap::right_const_iterator res = claimed_resources.right.find(resource);
            if (res != claimed_resources.right.end())
            {                                                       // resource is claimed
              srv.request.stop_controllers.push_back(res->second);  // add claiming controller to stop list
              claimed_resources.left.erase(res->second);            // remove claimed resources
            }
          }
        }
#endif
      }
    }
    srv.request.strictness = srv.request.STRICT;

    if (!srv.request.start_controllers.empty() || srv.request.stop_controllers.empty())
    {  // something to switch?
      if (!ros::service::call(getAbsName("controller_manager/switch_controller"), srv))
      {
        ROS_ERROR_STREAM("Could switch controllers at " << ns_);
      }
      discover(true);
      return srv.response.ok;
    }
    return true;
  }
};
}  // namespace moveit_ros_control_interface

PLUGINLIB_EXPORT_CLASS(moveit_ros_control_interface::MandroMoveItControllerManager,
                       moveit_controller_manager::MoveItControllerManager);