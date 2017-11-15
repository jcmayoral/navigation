#include <collision_detector_diagnoser/collision_detector_diagnoser.h>
#include <ros/ros.h>

using namespace collision_detector_diagnoser;

int main(int argc,char** argv){

  ros::init(argc, argv, "detector_diagnoser_node");

  int sensor_number = 4;

  if (argc >= 2 ){
    sensor_number = atoi(argv[1]);
  }

  ROS_INFO_STREAM("Setting " << sensor_number << "sensors");

  CollisionDetectorDiagnoser* diagnoser_ = new CollisionDetectorDiagnoser(sensor_number);

  while(ros::ok()){
    if(diagnoser_->detectFault()){
      ROS_INFO_STREAM("Collision Detected");
      diagnoser_->isolateFault();
    }
    //diagnoser_.isolateFault();
    ros::spinOnce(); // the missing call
  }

  //ros::spin();
  ROS_INFO("Out");

  return 1;
}
