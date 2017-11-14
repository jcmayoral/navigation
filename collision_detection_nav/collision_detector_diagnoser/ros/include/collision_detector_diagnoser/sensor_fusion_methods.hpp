#include <list>
#include <ros/ros.h>
#include <fusion_msgs/sensorFusionMsg.h>

using namespace std;

class SensorFusionApproach {
  public:
    bool detect(list<fusion_msgs::sensorFusionMsg> v) {
      ROS_INFO("Default");
      return false;
    };
};

class ConsensusApproach : public SensorFusionApproach {
    public:
    bool detect(list<fusion_msgs::sensorFusionMsg> v){
      ROS_INFO("Consensus");
      return 0;
    };
};
