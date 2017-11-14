#include "collision_detector_diagnoser/collision_detector_diagnoser.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

// register this class as a Fault Detector
PLUGINLIB_DECLARE_CLASS(collision_detector_diagnoser, CollisionDetectorDiagnoser,
                        collision_detector_diagnoser::CollisionDetectorDiagnoser,
                        fault_core::FaultDetector)

using namespace fault_core;
using namespace message_filters;
namespace collision_detector_diagnoser
{

  CollisionDetectorDiagnoser::CollisionDetectorDiagnoser(): isCollisionDetected(false), time_of_collision_(), mode_(0), sensor_number_(2)
  {
    ros::NodeHandle private_n;
    fault_.type_ =  FaultTopology::UNKNOWN_TYPE;
    fault_.cause_ = FaultTopology::UNKNOWN;
    strength_srv_client_ = private_n.serviceClient<kinetic_energy_monitor::KineticEnergyMonitorMsg>("kinetic_energy_drop");
    orientations_srv_client_ = private_n.serviceClient<footprint_checker::CollisionCheckerMsg>("collision_checker");
    dyn_server_cb = boost::bind(&CollisionDetectorDiagnoser::dyn_reconfigureCB, this, _1, _2);
    dyn_server.setCallback(dyn_server_cb);
    ROS_INFO("Default Constructor CollisionDetectorDiagnoser");
  }

  CollisionDetectorDiagnoser::CollisionDetectorDiagnoser(int sensor_number): isCollisionDetected(false), time_of_collision_(), mode_(0), sensor_number_(sensor_number)
  {
    ros::NodeHandle private_n;
    fault_.type_ =  FaultTopology::UNKNOWN_TYPE;
    fault_.cause_ = FaultTopology::UNKNOWN;
    strength_srv_client_ = private_n.serviceClient<kinetic_energy_monitor::KineticEnergyMonitorMsg>("kinetic_energy_drop");
    orientations_srv_client_ = private_n.serviceClient<footprint_checker::CollisionCheckerMsg>("collision_checker");
    dyn_server_cb = boost::bind(&CollisionDetectorDiagnoser::dyn_reconfigureCB, this, _1, _2);
    dyn_server.setCallback(dyn_server_cb);
    //initialize();
    ROS_INFO("Constructor CollisionDetectorDiagnoser");
  }

  void CollisionDetectorDiagnoser::dyn_reconfigureCB(collision_detector_diagnoser::dynamic_reconfigureConfig &config, uint32_t level){
    //ROS_INFO_STREAM(config.mode);
    mode_ = config.mode;
    initialize(sensor_number_);
  }

  CollisionDetectorDiagnoser::~CollisionDetectorDiagnoser()
  {

  }

  fault_core::FaultTopology CollisionDetectorDiagnoser::getFault()
  {
     return fault_;
  }

  void CollisionDetectorDiagnoser::twoSensorsCallBack(const fusion_msgs::sensorFusionMsgConstPtr& detector_1,
                                                      const fusion_msgs::sensorFusionMsgConstPtr& detector_2){
    ROS_INFO_ONCE("TwoSensors");
    list <fusion_msgs::sensorFusionMsg> list;
    fusion_msgs::sensorFusionMsg a = *detector_1;

    list.push_back(a);

    if(fusion_approach_.detect(list)){
      time_of_collision_ = detector_1->header; //TODO
      isCollisionDetected = true;

    }
    else{
      isCollisionDetected = false;
    }
  }

  void CollisionDetectorDiagnoser::threeSensorsCallBack(const fusion_msgs::sensorFusionMsgConstPtr& detector_1,
                                                        const fusion_msgs::sensorFusionMsgConstPtr& detector_2,
                                                        const fusion_msgs::sensorFusionMsgConstPtr& detector_3){
    ROS_INFO_ONCE("Three Sensors");

    list <fusion_msgs::sensorFusionMsg> list;
    fusion_msgs::sensorFusionMsg a = *detector_1;
    list.push_back(a);

    if(fusion_approach_.detect(list)){
      time_of_collision_ = detector_1->header; //TODO
      isCollisionDetected = true;

    }
    else{
      isCollisionDetected = false;
    }
  }

  void CollisionDetectorDiagnoser::simpleCallBack(const fusion_msgs::sensorFusionMsg msg){
    ROS_INFO_ONCE("Simple Filtering");
    if (msg.msg == fusion_msgs::sensorFusionMsg::ERROR){
      time_of_collision_ = msg.header;
      isCollisionDetected = true;

    }
    else{
      isCollisionDetected = false;
    }
  }

  void CollisionDetectorDiagnoser::initialize(int sensor_number)
  {
    ros::NodeHandle nh;
    ROS_INFO_STREAM("initializing " << sensor_number_ << " sensors");
    ROS_INFO_STREAM("Method" << std::to_string(mode_) << " Selected");

    switch(mode_){

      case 0:
        if (!filtered_subscribers_.empty()){
          //sub_0_->unsubscribe();
          //sub_1_->unsubscribe();
          //ROS_INFO("inside");
          for(int i=0; i< sensor_number; i++){
            filtered_subscribers_.at(i)->unsubscribe();
          }
          sync_->registerCallback(boost::bind(&CollisionDetectorDiagnoser::twoSensorsCallBack,this,_1, _2));
          filtered_subscribers_.clear();
        }

        for (int i = 1; i< sensor_number_;i++){
          ros::Subscriber sub = nh.subscribe("collisions_"+std::to_string(i), 10, &CollisionDetectorDiagnoser::simpleCallBack, this);
          array_subcribers_.push_back(sub);
        }

        break;

      case 1:

        for (int i = 0; i< array_subcribers_.size();i++){
          array_subcribers_[i].shutdown();
        }
        array_subcribers_.clear();

        for(int i=1; i< sensor_number; i++){
          filtered_subscribers_.push_back(new message_filters::Subscriber<fusion_msgs::sensorFusionMsg>(nh, "collisions_"+std::to_string(i), 10));
        }
        //typedef message_filters::sync_policies::ApproximateTime<fusion_msgs::sensorFusionMsg, fusion_msgs::sensorFusionMsg> MySyncPolicy;
        sync_ = new message_filters::Synchronizer<MySyncPolicy2>(MySyncPolicy2(10),*filtered_subscribers_.at(0), *filtered_subscribers_.at(1));
        sync_->registerCallback(boost::bind(&CollisionDetectorDiagnoser::twoSensorsCallBack,this,_1, _2));
        break;

      default:
        ROS_ERROR("Method can not be changed");
        break;
    }
  }

  bool CollisionDetectorDiagnoser::detectFault()
  {
    ROS_DEBUG("SimpleCollisionDetector Detect Fault");
    //if (isCollisionDetected){
    // isolateFault();
    //}
    return isCollisionDetected;
  }

  void CollisionDetectorDiagnoser::isolateFault(){

    footprint_checker::CollisionCheckerMsg srv;

    if(orientations_srv_client_.call(srv)){
      ROS_INFO("Orientations Computed Correctly");
    }
    else{
      ROS_WARN("Error in orientations Server");
    }

    fault_.type_ = FaultTopology::COLLISION;
    ROS_INFO("Isolating Platform Collision");
    diagnoseFault();
  }

  void CollisionDetectorDiagnoser::diagnoseFault(){
    //Force
    kinetic_energy_monitor::KineticEnergyMonitorMsg srv;
    srv.request.collision_time = time_of_collision_;

    if(strength_srv_client_.call(srv)){
      ROS_INFO_STREAM("Strength: " << srv.response.energy_lost);
    }

    fault_.cause_ = FaultTopology::MISLOCALIZATION;
    ROS_ERROR_ONCE("Collision FOUND");
    isCollisionDetected = false;
  }
}  // namespace collision_detector_diagnoser
