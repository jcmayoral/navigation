/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*
* Author: Eitan Marder-Eppstein
* Modified by: Jose Mayoral
*********************************************************************/
#ifndef NAV_FAULT_TOLERANT_MOVE_BASE_ACTION_H_
#define NAV_FAULT_TOLERANT_MOVE_BASE_ACTION_H_

#include <vector>
#include <string>
#include <ros/ros.h>
#include <move_base/move_base.h>
#include <move_base/move_base_states.h>
#include <fault_core/fault_detector.h>


using namespace move_base;
namespace move_base_fault_tolerant {
  /**
   * @class FaultTolerantMoveBase
   * @brief A class that uses the actionlib::ActionServer interface that moves the robot base to a goal location.
   */

   class FaultTolerantMoveBase: public MoveBase{

     public:
       /**
        * @brief  Performs a control cycle
        * @param goal A reference to the goal to pursue
        * @param global_plan A reference to the global plan being used
        * @return True if processing of the goal is done, false otherwise
        * @Override from MoveBase
        */

      virtual bool executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan);
      /**
       * @brief  Constructor for the actions
       * @param name The name of the action
       * @param tf A reference to a TransformListener
       */
       FaultTolerantMoveBase(tf::TransformListener& tf);
      /**
       * @brief  Destructor - Cleans up
       */
      virtual ~FaultTolerantMoveBase();

      /**
       * @brief  Do Planning Machine State

       */

     private:

       //Containers
       std::string fault_detector_;

       //FaultDetector
       pluginlib::ClassLoader<fault_core::FaultDetector> fd_loader_;
       boost::shared_ptr<fault_core::FaultDetector> fd_;
       void detectFault();
       void recoveryFault();
       void createFaultDetector();
       boost::thread* detection_thread_;
       // For detection thread
       boost::mutex detection_mutex_;

  };
};
#endif
