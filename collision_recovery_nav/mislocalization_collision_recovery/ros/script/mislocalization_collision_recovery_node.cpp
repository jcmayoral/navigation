#include <mislocalization_collision_recovery/mislocalization_collision_recovery.h>
#include <ros/ros.h>

using namespace mislocalization_collision_recovery;

int main(int argc,char** argv){

  ros::init(argc, argv, "mislocalization_collision_recovery_node");

  MisLocalizationCollisionRecovery* strategy = new MisLocalizationCollisionRecovery();

  while(ros::ok()){

    if(strategy->runFaultBehavior()){
      ROS_INFO("DONE");
      return 1;
    }
    ros::spinOnce(); // the missing call
    //diagnoser_.isolateFault();
  }

  return 1;
}
