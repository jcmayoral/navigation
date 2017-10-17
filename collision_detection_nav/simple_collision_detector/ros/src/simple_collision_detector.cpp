#include "simple_collision_detector/simple_collision_detector.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

// register this class as a Fault Detector
PLUGINLIB_DECLARE_CLASS(simple_collision_detector, SimpleCollisionDetector,
                        simple_collision_detector::SimpleCollisionDetector,
                        fault_core::FaultDetector)

using namespace fault_core;
using namespace message_filters;
namespace simple_collision_detector
{

  SimpleCollisionDetector::SimpleCollisionDetector(): isCollisionDetected(false)
  {
    fault_.type_ =  FaultTopology::UNKNOWN_TYPE;
    fault_.cause_ = FaultTopology::UNKNOWN;
    ROS_INFO("Constructor SimpleCollisionDetector");
  }


  SimpleCollisionDetector::~SimpleCollisionDetector()
  {

  }

  fault_core::FaultTopology SimpleCollisionDetector::getFault()
  {
     return fault_;
  }


  void SimpleCollisionDetector::mainCallBack(const fusion_msgs::sensorFusionMsg msg){
    ROS_DEBUG_STREAM("Message received " << msg.window_size);
    if (msg.msg == fusion_msgs::sensorFusionMsg::ERROR){
      isCollisionDetected = true;
    }
    else{
      isCollisionDetected = false;
    }
  }

  /*void SimpleCollisionDetector::secondCallBack(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr&  msg2){
    ROS_INFO("secondCallBack");
  }*/
  /*void SimpleCollisionDetector::thirdCallBack(const  geometry_msgs::AccelStamped::ConstPtr& msg){
    ROS_INFO("third CB");
  }*/


  void SimpleCollisionDetector::initialize(int sensor_number)
  {
    ros::NodeHandle nh;
    ROS_INFO_STREAM("initializing " << sensor_number << " sensors");
    for (int i = 0; i< sensor_number;i++){
      ros::Subscriber sub = nh.subscribe("collisions_"+std::to_string(i), 10, &SimpleCollisionDetector::mainCallBack, this);
      array_subcribers_.push_back(sub);
    }

    /*
    //Sequencer
    message_filters::Subscriber<geometry_msgs::AccelStamped> sub(nh, "my_topic", 1);
    message_filters::TimeSequencer<geometry_msgs::AccelStamped> seq(sub, ros::Duration(0.1), ros::Duration(0.01), 10);
    seq.registerCallback(boost::bind(&SimpleCollisionDetector::thirdCallBack,this,_1));

    //Syncronizer
    message_filters::Subscriber<sensor_msgs::Image> sub(nh, "my_topic", 1);
    message_filters::Subscriber<sensor_msgs::Image> sub2(nh, "my_topic2", 1);
    TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(sub, sub2, 10);
    sync.registerCallback(boost::bind(&SimpleCollisionDetector::secondCallBack,this, _1, _2));
    ros::spin();
    */
  }

  bool SimpleCollisionDetector::detectFault()
  {
    ROS_DEBUG("SimpleCollisionDetector Detect Fault");
    if (isCollisionDetected){
      isolateFault();
    }
    return isCollisionDetected;
  }

  void SimpleCollisionDetector::isolateFault(){
    ROS_INFO("Isolating Platform Collision");
    diagnoseFault();
  }

  void SimpleCollisionDetector::diagnoseFault(){
    fault_.cause_ = FaultTopology::MISLOCALIZATION;
    fault_.type_ = FaultTopology::COLLISION;
    ROS_ERROR_ONCE("Collision FOUND");
  }
}  // namespace simple_collision_detector
