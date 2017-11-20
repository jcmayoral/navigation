#include <mislocalization_collision_recovery/mislocalization_collision_recovery.h>
#include <ros/ros.h>

using namespace mislocalization_collision_recovery;

int main(int argc,char** argv){

  ros::init(argc, argv, "mislocalization_collision_recovery_node");

  MisLocalizationCollisionRecovery* strategy = new MisLocalizationCollisionRecovery();

  while(ros::ok()){
    while(!strategy->runFaultBehavior());
    //diagnoser_.isolateFault();
    ros::spinOnce(); // the missing call
  }

  return 1;
}
