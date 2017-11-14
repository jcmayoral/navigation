#include <list>
#include <ros/ros.h>
#include <fusion_msgs/sensorFusionMsg.h>

using namespace std;

class SensorFusionApproach {
  public:
    virtual bool detect(list<fusion_msgs::sensorFusionMsg> v) {
      ROS_DEBUG("Default");
      return false;
    };
};

class ConsensusApproach : public SensorFusionApproach {
    public:
    bool detect(list<fusion_msgs::sensorFusionMsg> v){
      ROS_DEBUG("Consensus");
      return 0;
    };
};
