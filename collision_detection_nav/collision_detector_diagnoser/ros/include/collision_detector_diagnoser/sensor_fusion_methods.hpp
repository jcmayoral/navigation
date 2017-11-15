#include <list>
#include <ros/ros.h>
#include <fusion_msgs/sensorFusionMsg.h>

using namespace std;

class SensorFusionApproach {
  public:
    virtual bool detect(list<fusion_msgs::sensorFusionMsg> v) {
      ROS_DEBUG("Default");
      for (std::list<fusion_msgs::sensorFusionMsg>::iterator it=v.begin(); it != v.end(); ++it){
        if(it->msg == fusion_msgs::sensorFusionMsg::ERROR){
          return true;
        }
      }
      return false;
    };
  protected:
    double threshold = 0.5; //TODO
};

class ConsensusApproach : public SensorFusionApproach {
    public:
      bool detect(list<fusion_msgs::sensorFusionMsg> v){
        ROS_DEBUG("Consensus");
        int counter = 0;
        for (std::list<fusion_msgs::sensorFusionMsg>::iterator it=v.begin(); it != v.end(); ++it){
          if(it->msg == fusion_msgs::sensorFusionMsg::ERROR){
            ROS_WARN_STREAM("Collision DETECTED on " << it->sensor_id);
            counter ++;
          }
        }
        ROS_DEBUG_STREAM("Counter " << counter);
        if (counter >= v.size()*threshold){
          return true;

        }

        return false;
      };
};
