/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  are met:
*  modification, are permitted provided that the following conditions
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
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN(
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein""
*         Mike Phillips (put the planner in its own thread)
*         Jose Mayoral (fault tolerant extension)
*********************************************************************/
#include <move_base_fault_tolerant/move_base_fault_tolerant.h>
#include <move_base/move_base.h>

using namespace move_base;

namespace move_base_fault_tolerant {

  FaultTolerantMoveBase::FaultTolerantMoveBase(tf::TransformListener& tf):
    MoveBase(tf),fd_loader_("fault_core", "fault_core::FaultDetector"), fault_recovery_loader_("fault_core", "fault_core::FaultRecoveryBehavior")
    {
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    private_nh.param("fault_detector", fault_detector_, std::string("simple_collision_detector/SimpleCollisionDetector"));
    ROS_INFO_STREAM ("Selected Fault Detector: " << fault_detector_);
    createFaultDetector();

    if(!loadFaultRecoveryBehaviors(private_nh)){
      ROS_WARN("No Fault Recovery Behaviors");
    }

    detection_thread_ = new boost::thread(boost::bind(&FaultTolerantMoveBase::detectFault, this));
    ROS_INFO("FaultTolerantMoveBase Initialized");

  }

//Based on Global and Local Planner instantiations
  void FaultTolerantMoveBase::createFaultDetector(){
    //create a FaultDetector
    try {
      //check if a non fully qualified name has potentially been passed in
      if(!fd_loader_.isClassAvailable(fault_detector_)){
        std::vector<std::string> classes = fd_loader_.getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i){
          if(fault_detector_ == fd_loader_.getName(classes[i])){
            //if we've found a match... we'll get the fully qualified name and break out of the loop
            ROS_WARN("Fault Detector specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                fault_detector_.c_str(), classes[i].c_str());
            fault_detector_ = classes[i];
            break;
          }
        }
      }

      fd_ = fd_loader_.createInstance(fault_detector_);
      ROS_INFO("Created fault_detector_ %s", fault_detector_.c_str());
      //fd_->initialize(fd_loader_.getName(fault_detector_));
      fd_->initialize(1);
    } catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the %s detector, are you sure it is properly registered and that the containing library is built? Exception: %s", fault_detector_.c_str(), ex.what());
      exit(1);
    }

  }

  FaultTolerantMoveBase::~FaultTolerantMoveBase(){
    detection_thread_->interrupt();
    detection_thread_->join();
    delete detection_thread_;
  }

  void FaultTolerantMoveBase::detectFault(){
    ros::NodeHandle n;
    boost::unique_lock<boost::mutex> lock(detection_mutex_);
    int counter = 0;
    while(n.ok()){
      if (fd_->detectFault())
      {
        setState(MoveBaseState::RECOVERING);
      }
      else{
        ROS_DEBUG("Healthy State");
      }
    }
    lock.unlock();
  }

  void FaultTolerantMoveBase::recoveryFault(){
    ROS_DEBUG("recoveryFault");
  }

  bool FaultTolerantMoveBase::loadFaultRecoveryBehaviors(ros::NodeHandle node){
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("fault_recovery_behaviors", behavior_list)){
      if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray){
        for(int i = 0; i < behavior_list.size(); ++i){
          if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct){
            if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")){
              //check for recovery behaviors with the same name
              for(int j = i + 1; j < behavior_list.size(); j++){
                if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct){
                  if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")){
                    std::string name_i = behavior_list[i]["name"];
                    std::string name_j = behavior_list[j]["name"];
                    if(name_i == name_j){
                      ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                          name_i.c_str());
                      return false;
                    }
                  }
                }
              }
            }
            else{
              ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
              return false;
            }
          }
          else{
            ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
            return false;
          }
        }

        //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
        for(int i = 0; i < behavior_list.size(); ++i){
          try{
            //check if a non fully qualified name has potentially been passed in
            if(!fault_recovery_loader_.isClassAvailable(behavior_list[i]["type"])){
              std::vector<std::string> classes = fault_recovery_loader_.getDeclaredClasses();
              for(unsigned int i = 0; i < classes.size(); ++i){
                if(behavior_list[i]["type"] == fault_recovery_loader_.getName(classes[i])){
                  //if we've found a match... we'll get the fully qualified name and break out of the loop
                  ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                      std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                  behavior_list[i]["type"] = classes[i];
                  break;
                }
              }
            }

            boost::shared_ptr<fault_core::FaultRecoveryBehavior> behavior(fault_recovery_loader_.createInstance(behavior_list[i]["type"]));

            //shouldn't be possible, but it won't hurt to check
            if(behavior.get() == NULL){
              ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
              return false;
            }

            //initialize the recovery behavior with its name
            behavior->initialize(planner_costmap_ros_, controller_costmap_ros_);
            fault_recovery_behaviors_.push_back(behavior);
          }
          catch(pluginlib::PluginlibException& ex){
            ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
            return false;
          }
        }
      }
      else{
        ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
            behavior_list.getType());
        return false;
      }
    }
    else{
      //if no recovery_behaviors are specified, we'll just load the defaults
      return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
  }

  bool FaultTolerantMoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
    ROS_DEBUG("fault executeCycle");
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;

    //update feedback to correspond to our curent position
    tf::Stamped<tf::Pose> global_pose;
    planner_costmap_ros_->getRobotPose(global_pose);
    geometry_msgs::PoseStamped current_position;
    tf::poseStampedTFToMsg(global_pose, current_position);

    //push the feedback out
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = current_position;
    as_->publishFeedback(feedback);

    //check to see if we've moved far enough to reset our oscillation timeout
    //if(distance(current_position, oscillation_pose_) >= getOscillationDistance())
    if(distance(current_position, getOscillationPose()) >= getOscillationDistance())
    {
      //last_oscillation_reset_ = ros::Time::now();
      setTime(ros::Time::now(),2);
      //oscillation_pose_ = current_position;
      setOscillationPose(current_position);
      //if our last recovery was caused by oscillation, we want to reset the recovery index
      if(getRecoveryTrigger() == MoveBaseState::OSCILLATION_R)
        setRecoveryIndex(0);
    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    if(!controller_costmap_ros_->isCurrent()){
      ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
      publishZeroVelocity();
      return false;
    }

    //if we have a new plan then grab it and give it to the controller
    if(isNewGlobalPlanAvailable()){
      //make sure to set the new plan flag to false
      setNewGlobalPlan(false);

      ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

      //do a pointer swap under mutex
      //std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;
      std::vector<geometry_msgs::PoseStamped>* temp_plan = getPlan(2);

      boost::unique_lock<boost::mutex> lock(planner_mutex_);
      //controller_plan_ = latest_plan_;
      setNewPlan(getPlan(1),2);
      //latest_plan_ = temp_plan;
      setNewPlan(temp_plan,1);
      lock.unlock();
      ROS_DEBUG_NAMED("move_base","pointers swapped!");

      //if(!tc_->setPlan(*controller_plan_)){
      if(!tc_->setPlan(*getPlan(2))){
        //ABORT and SHUTDOWN COSTMAPS
        ROS_ERROR("Failed to pass global plan to the controller, aborting.");
        resetState();

        //disable the planner thread
        lock.lock();
        setRunPlanner(false);
        lock.unlock();

        as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
        return true;
      }

      //make sure to reset recovery_index_ since we were able to find a valid plan
      if(getRecoveryTrigger() == MoveBaseState::PLANNING_R)
        setRecoveryIndex(0);
    }

    /*
    * THIS IS ACTUALLY A STATE machine
    * Review notes of the state machine to decide how to modify
    */
    //the move_base state machine, handles the control logic for navigation
    //ROS_INFO_STREAM("State " << getState());
    switch(getState()){
      //if we are in a planning state, then we'll attempt to make a plan
      case MoveBaseState::PLANNING:
        {
          boost::mutex::scoped_lock lock(planner_mutex_);
          setRunPlanner(true);
          planner_cond_.notify_one();
        }
        ROS_DEBUG_NAMED("move_base","Waiting for plan, in the planning state.");
        break;

      //if we're controlling, we'll attempt to find valid velocity commands
      case MoveBaseState::CONTROLLING:
        ROS_DEBUG_NAMED("move_base","In controlling state.");

        //FAULT DETECTION
        //FaultTolerantMoveBase::detectFault();
        ROS_DEBUG_STREAM("State " << getState());

        //check to see if we've reached our goal
        if(tc_->isGoalReached()){
          ROS_DEBUG_NAMED("move_base","Goal reached!");
          resetState();

          //disable the planner thread
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          setRunPlanner(false);
          lock.unlock();

          as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
          return true;
        }

        //check for an oscillation condition
        if(getOscillationTimeout() > 0.0 &&
            getTime(2) + ros::Duration(getOscillationTimeout()) < ros::Time::now())
        {
          publishZeroVelocity();
          setState(MoveBaseState::CLEARING);
          setRecoveryTrigger(MoveBaseState::OSCILLATION_R);
        }

        {
         boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

        if(tc_->computeVelocityCommands(cmd_vel)){
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          //last_valid_control_ = ros::Time::now();
          setTime(ros::Time::now(),1);
          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);
          if(getRecoveryTrigger() == MoveBaseState::CONTROLLING_R)
            setRecoveryIndex(0);
        }
        else {
          ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
          //ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);
          ros::Time attempt_end = getTime(1) + ros::Duration(getControllerPatience());

          //check if we've tried to find a valid control for longer than our time limit
          if(ros::Time::now() > attempt_end){
            //we'll move into our obstacle clearing mode
            publishZeroVelocity();
            setState(MoveBaseState::CLEARING);
            setRecoveryTrigger(move_base::MoveBaseState::CONTROLLING_R);
          }
          else{
            //otherwise, if we can't find a valid control, we'll go back to planning
            //last_valid_plan_ = ros::Time::now();
            setTime(ros::Time::now(), 0);
            planning_retries_ = 0;
            setState(MoveBaseState::PLANNING);
            publishZeroVelocity();

            //enable the planner thread in case it isn't running on a clock
            boost::unique_lock<boost::mutex> lock(planner_mutex_);
            setRunPlanner(true);
            planner_cond_.notify_one();
            lock.unlock();
          }
        }
        }

        break;

      //we'll try to clear out space with any user-provided recovery behaviors
      case MoveBaseState::CLEARING:
        ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
        //we'll invoke whatever recovery behavior we're currently on if they're enabled
        if(isRecoveryBehaviorEnabled() && getRecoveryIndex() < recovery_behaviors_.size()){
          ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", getRecoveryIndex(), recovery_behaviors_.size());
          recovery_behaviors_[getRecoveryIndex()]->runBehavior();

          //we at least want to give the robot some time to stop oscillating after executing the behavior
          //last_oscillation_reset_ = ros::Time::now();
          setTime(ros::Time::now(),2);

          //we'll check if the recovery behavior actually worked
          ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
          setState(MoveBaseState::PLANNING);

          //update the index of the next recovery behavior that we'll try
          setRecoveryIndex(1,true);
        }
        else{
          ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");
          //disable the planner thread
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          setRunPlanner(false);
          lock.unlock();

          ROS_DEBUG_NAMED("move_base_recovery","Something should abort after this.");

          if(getRecoveryTrigger() == MoveBaseState::CONTROLLING_R){
            ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid control. Even after executing recovery behaviors.");
          }
          else if(getRecoveryTrigger() == MoveBaseState::PLANNING_R){
            ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to find a valid plan. Even after executing recovery behaviors.");
          }
          else if(getRecoveryTrigger() == MoveBaseState::OSCILLATION_R){
            ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
            as_->setAborted(move_base_msgs::MoveBaseResult(), "Robot is oscillating. Even after executing recovery behaviors.");
          }
          resetState();
          return true;
        }
        break;

      case MoveBaseState::RECOVERING:
      {
        //Following Clearing State "template"
        //Disable Planner_thread
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        setRunPlanner(false);
        lock.unlock();

        ROS_ERROR("move_base in state RECOVERING");
        FaultTolerantMoveBase::recoveryFault();
        //publishZeroVelocity();

        for (int i = 0; i<fault_recovery_behaviors_.size(); i++){
          if (fd_->getFault().cause_ == fault_recovery_behaviors_[i]->getType()){
            fault_recovery_behaviors_[i]->runFaultBehavior();
          }
        }
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Collision Recovery Failure.");
        resetState();
        return true;
        break;
      }

      default:
        ROS_ERROR("This case should never be reached, something is wrong, aborting");
        resetState();
        //disable the planner thread
        boost::unique_lock<boost::mutex> lock(planner_mutex_);
        setRunPlanner(false);
        lock.unlock();
        as_->setAborted(move_base_msgs::MoveBaseResult(), "Reached a case that should not be hit in move_base. This is a bug, please report it.");
        return true;
    }

    //we aren't done yet
    return false;
  }
};
