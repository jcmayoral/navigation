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

  void CollisionDetectorDiagnoser::plotOrientation(list<fusion_msgs::sensorFusionMsg> v){
    geometry_msgs::PoseArray array_msg;
    array_msg.header.frame_id = "base_link";

    for (std::list<fusion_msgs::sensorFusionMsg>::iterator it=v.begin(); it != v.end(); ++it){
      geometry_msgs::Pose pose;
      pose.orientation = tf::createQuaternionMsgFromYaw (it->angle);
      array_msg.poses.push_back(pose);
      //if(it->msg == fusion_msgs::sensorFusionMsg::ERROR){
      //  return true;
      //}
    }

    orientation_pub_.publish(array_msg);

  }


  CollisionDetectorDiagnoser::CollisionDetectorDiagnoser(): isCollisionDetected(false), time_of_collision_(), mode_(0), sensor_number_(4), filter_(false), percentage_threshold_(0.5)
  {
    ros::NodeHandle private_n("~");
    fault_.type_ =  FaultTopology::UNKNOWN_TYPE;
    fault_.cause_ = FaultTopology::UNKNOWN;
    strength_srv_client_ = private_n.serviceClient<kinetic_energy_monitor::KineticEnergyMonitorMsg>("/kinetic_energy_drop");
    orientations_srv_client_ = private_n.serviceClient<footprint_checker::CollisionCheckerMsg>("/collision_checker");
    speak_pub_ = private_n.advertise<std_msgs::String>("/say",1);
    ros::Duration(2).sleep();

    /*while (speak_pub_.getNumSubscribers() < 1){
      ROS_INFO_ONCE("Waiting Subscriber for say server");
    }*/

    orientation_pub_ = private_n.advertise<geometry_msgs::PoseArray>("measured_collision_orientations", 1);

    private_n.param("sensor_fusion/sensor_number", sensor_number_, 1);
    private_n.param("sensor_fusion/mode", mode_, 1);
    private_n.param("sensor_fusion/filter", filter_, true);
    ROS_INFO_STREAM("mode" << mode_);
    initialize(sensor_number_);

    setDynamicReconfigureServer();
    ROS_INFO("Default Constructor CollisionDetectorDiagnoser");
  }

  CollisionDetectorDiagnoser::CollisionDetectorDiagnoser(int sensor_number): isCollisionDetected(false), time_of_collision_(), mode_(0), sensor_number_(sensor_number), filter_(false)
  {
    //Used In teh Node
    ros::NodeHandle private_n("~");
    fault_.type_ =  FaultTopology::UNKNOWN_TYPE;
    fault_.cause_ = FaultTopology::UNKNOWN;
    strength_srv_client_ = private_n.serviceClient<kinetic_energy_monitor::KineticEnergyMonitorMsg>("kinetic_energy_drop");
    orientations_srv_client_ = private_n.serviceClient<footprint_checker::CollisionCheckerMsg>("collision_checker");
    speak_pub_ = private_n.advertise<std_msgs::String>("/say",1);
    ros::Duration(2).sleep();

    /*while (speak_pub_.getNumSubscribers() < 1){
      ROS_INFO_ONCE("Waiting Subscriber for say server");
    }*/

    orientation_pub_ = private_n.advertise<geometry_msgs::PoseArray>("measured_collision_orientations", 1);
    setDynamicReconfigureServer();

    //initialize();
    ROS_INFO("Constructor CollisionDetectorDiagnoser");
  }

  void CollisionDetectorDiagnoser::setDynamicReconfigureServer(){
    std::string node_handler_name("collision_detector_diagnoser");
    ros::NodeHandle nh_for_dyn("~/"+ node_handler_name);
    dyn_server = new dynamic_reconfigure::Server<collision_detector_diagnoser::diagnoserConfig>(nh_for_dyn);
    dyn_server_cb = boost::bind(&CollisionDetectorDiagnoser::dyn_reconfigureCB, this, _1, _2);
    dyn_server->setCallback(dyn_server_cb);
  }

  void CollisionDetectorDiagnoser::dyn_reconfigureCB(collision_detector_diagnoser::diagnoserConfig &config, uint32_t level){
    //<ROS_INFO_STREAM(config.mode);
    mode_ = config.mode;
    filter_ = config.allow_filter;
    percentage_threshold_ = config.percentage_threshold;
    sensor_number_ = config.sensor_sources;
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
    ROS_INFO("TwoSensors");
    list <fusion_msgs::sensorFusionMsg> list;
    fusion_msgs::sensorFusionMsg tmp = *detector_1;
    list.push_back(tmp);
    tmp = *detector_2;
    list.push_back(tmp);

    if(fusion_approach_->detect(list)){
      plotOrientation(list);
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
    ROS_INFO("Three Sensors");

    list <fusion_msgs::sensorFusionMsg> list;
    fusion_msgs::sensorFusionMsg tmp = *detector_1;
    list.push_back(tmp);
    tmp = *detector_2;
    list.push_back(tmp);
    tmp = *detector_3;
    list.push_back(tmp);

    if(fusion_approach_->detect(list)){
      plotOrientation(list);
      time_of_collision_ = detector_1->header; //TODO
      isCollisionDetected = true;

    }
    else{
      isCollisionDetected = false;
    }
  }

  void CollisionDetectorDiagnoser::fourSensorsCallBack(const fusion_msgs::sensorFusionMsgConstPtr& detector_1,
                                                        const fusion_msgs::sensorFusionMsgConstPtr& detector_2,
                                                        const fusion_msgs::sensorFusionMsgConstPtr& detector_3,
                                                        const fusion_msgs::sensorFusionMsgConstPtr& detector_4){
    ROS_INFO("Four Sensors");

    list <fusion_msgs::sensorFusionMsg> list;
    fusion_msgs::sensorFusionMsg tmp = *detector_1;
    list.push_back(tmp);
    tmp = *detector_2;
    list.push_back(tmp);
    tmp = *detector_3;
    list.push_back(tmp);
    tmp = *detector_4;
    list.push_back(tmp);

    if(fusion_approach_->detect(list)){
      plotOrientation(list);
      time_of_collision_ = detector_1->header; //TODO
      isCollisionDetected = true;

    }
    else{
      isCollisionDetected = false;
    }
  }

  void CollisionDetectorDiagnoser::fiveSensorsCallBack(const fusion_msgs::sensorFusionMsgConstPtr& detector_1,
                                                        const fusion_msgs::sensorFusionMsgConstPtr& detector_2,
                                                        const fusion_msgs::sensorFusionMsgConstPtr& detector_3,
                                                        const fusion_msgs::sensorFusionMsgConstPtr& detector_4,
                                                        const fusion_msgs::sensorFusionMsgConstPtr& detector_5){
    ROS_INFO("Five Sensors");

    list <fusion_msgs::sensorFusionMsg> list;
    fusion_msgs::sensorFusionMsg tmp = *detector_1;
    list.push_back(tmp);
    tmp = *detector_2;
    list.push_back(tmp);
    tmp = *detector_3;
    list.push_back(tmp);
    tmp = *detector_4;
    list.push_back(tmp);
    tmp = *detector_5;
    list.push_back(tmp);

    if(fusion_approach_->detect(list)){
      plotOrientation(list);
      time_of_collision_ = detector_1->header; //TODO
      isCollisionDetected = true;

    }
    else{
      isCollisionDetected = false;
    }
  }


  void CollisionDetectorDiagnoser::simpleCallBack(const fusion_msgs::sensorFusionMsg msg){
    ROS_INFO("Simple Filtering");

    list <fusion_msgs::sensorFusionMsg> list;
    fusion_msgs::sensorFusionMsg tmp = msg;
    list.push_back(msg);

    //FOR TESTING
    if(fusion_approach_->detect(list)){
      plotOrientation(list);

    }
    //end for testing

    if (msg.msg == fusion_msgs::sensorFusionMsg::ERROR){
      ROS_INFO_STREAM ("Collision detected by " << msg.sensor_id);
      time_of_collision_ = msg.header;
      isCollisionDetected = true;

    }
    else{
      isCollisionDetected = false;
    }
  }

  void CollisionDetectorDiagnoser::unregisterCallbackForSyncronizers(){
    main_connection.disconnect();
  }

  void CollisionDetectorDiagnoser::registerCallbackForSyncronizers(int sensor_number){

    ROS_INFO_STREAM("Set CB Sync");
    switch(sensor_number){

      case 2:
        syncronizer_for_two_ = new message_filters::Synchronizer<MySyncPolicy2>(MySyncPolicy2(10), *filtered_subscribers_.at(0),
                                                                                               *filtered_subscribers_.at(1));
        main_connection = syncronizer_for_two_->registerCallback(boost::bind(&CollisionDetectorDiagnoser::twoSensorsCallBack,this,_1, _2));
        break;
      case 3:
        syncronizer_for_three_ = new message_filters::Synchronizer<MySyncPolicy3>(MySyncPolicy3(10), *filtered_subscribers_.at(0),
                                                                                                     *filtered_subscribers_.at(1),
                                                                                                     *filtered_subscribers_.at(2));
        main_connection = syncronizer_for_three_->registerCallback(boost::bind(&CollisionDetectorDiagnoser::threeSensorsCallBack,this,_1, _2,_3));
        break;
      case 4:
        syncronizer_for_four_ = new message_filters::Synchronizer<MySyncPolicy4>(MySyncPolicy4(10), *filtered_subscribers_.at(0),
                                                                                               *filtered_subscribers_.at(1),
                                                                                               *filtered_subscribers_.at(2),
                                                                                               *filtered_subscribers_.at(3));
        main_connection = syncronizer_for_four_->registerCallback(boost::bind(&CollisionDetectorDiagnoser::fourSensorsCallBack,this,_1, _2,_3,_4));
        break;
      case 5:
        syncronizer_for_five_ = new message_filters::Synchronizer<MySyncPolicy5>(MySyncPolicy5(10), *filtered_subscribers_.at(0),
                                                                                             *filtered_subscribers_.at(1),
                                                                                             *filtered_subscribers_.at(2),
                                                                                             *filtered_subscribers_.at(3),
                                                                                             *filtered_subscribers_.at(4));
        main_connection = syncronizer_for_five_->registerCallback(boost::bind(&CollisionDetectorDiagnoser::fiveSensorsCallBack,this,_1, _2,_3,_4,_5));
        break;

      default:
      ROS_ERROR_STREAM("registerCallback Failure");
    }
  }


  void CollisionDetectorDiagnoser::resetUnFilteredPublishers(){
    ROS_INFO("Reset Unfilter");
    for (int i = 0; i< array_subcribers_.size();i++){//remove normal subscribers
      array_subcribers_[i].shutdown();
    }//endFor
    array_subcribers_.clear();

  }

  void CollisionDetectorDiagnoser::resetFilteredPublishers(){
    ROS_INFO("Reset Filtered");
    for(int i=0; i< filtered_subscribers_.size(); i++){
      filtered_subscribers_.at(i)->unsubscribe();// unsubscribe all filtered messages
    }//endFor

    //registerCallbackForSyncronizers(sensor_number); //initiate the syncronizers which belongs to the selection
    filtered_subscribers_.clear();
  }

  void CollisionDetectorDiagnoser::setUnfilteredPublishers(int sensor_number, ros::NodeHandle nh){
    ROS_INFO("Set Unfiltered Publishers");
    for (int i = 0; i< sensor_number;i++){//add normal subscribers
      ros::Subscriber sub = nh.subscribe("collisions_"+std::to_string(i), 10, &CollisionDetectorDiagnoser::simpleCallBack, this);
      array_subcribers_.push_back(sub);
    }//
  }

  void CollisionDetectorDiagnoser::setFilteredPublishers(int sensor_number, ros::NodeHandle nh){
    ROS_INFO("Set Filtered Publishers");

    for(int i=0; i< sensor_number; i++){// create subscribers to filter
      filtered_subscribers_.push_back(new message_filters::Subscriber<fusion_msgs::sensorFusionMsg>(nh, "collisions_"+std::to_string(i), 10));
    }
    registerCallbackForSyncronizers(sensor_number); //initiate the syncronizers which belongs to the selection
  }

  void CollisionDetectorDiagnoser::initialize(int sensor_number)
  {
    ros::NodeHandle nh;
    sensor_number_ = sensor_number;
    ROS_INFO_STREAM("initializing " << sensor_number << " sensors");
    ROS_INFO_STREAM("Method" << std::to_string(mode_) << " Selected");

    if(!filter_){//unfiltered
      resetFilteredPublishers();
      setUnfilteredPublishers(sensor_number, nh);
    }//Endif

    else{// Switching to filtered messages
      resetUnFilteredPublishers();
      unregisterCallbackForSyncronizers();
      setFilteredPublishers(sensor_number, nh);
    }//endElse

    //Swap betweenModes;
    selectMode();

    //Update Threshold
    fusion_approach_->setThreshold(percentage_threshold_);

  }

  void CollisionDetectorDiagnoser::selectMode(){
    switch(mode_){
      case 0: fusion_approach_ = &default_approach_; break;
      case 1: fusion_approach_ = &consensus_approach_; break;
      case 2: fusion_approach_ = &weighted_approach_; break;
      case 3: fusion_approach_ = &kalman_approach_; break;
      default: ROS_ERROR_STREAM("selecMode Error with code "<< mode_);break;
    }//endSwitch
  }

  bool CollisionDetectorDiagnoser::detectFault()
  {
    //ROS_DEBUG("SimpleCollisionDetector Detect Fault");
    //if (isCollisionDetected){
    // isolateFault();
    //}
    return isCollisionDetected;
  }

  void CollisionDetectorDiagnoser::isolateFault(){

    footprint_checker::CollisionCheckerMsg srv;

    std_msgs::String msg;
    msg.data ="ouch";
    speak_pub_.publish(msg);


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

    //fault_.cause_ = FaultTopology::MISLOCALIZATION;
    fault_.cause_ = FaultTopology::STATIC_OBSTACLE;
    ROS_ERROR_ONCE("Collision FOUND");
    isCollisionDetected = false;
  }
}  // namespace collision_detector_diagnoser
