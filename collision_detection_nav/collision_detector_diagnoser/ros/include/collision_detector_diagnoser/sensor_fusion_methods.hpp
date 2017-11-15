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
    void setThreshold(double thr){
      ROS_INFO("Threshold update");
      threshold = thr;
    }
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

class WeightedApproach : public SensorFusionApproach {
    public:
      bool detect(list<fusion_msgs::sensorFusionMsg> v){
        ROS_DEBUG("Consensus");
        double max_value = v.size() * fusion_msgs::sensorFusionMsg::ERROR; //TODO
        double count = 0;

        for (std::list<fusion_msgs::sensorFusionMsg>::iterator it=v.begin(); it != v.end(); ++it){
          count += it->msg * it->weight;
        }

        ROS_DEBUG_STREAM("Count " << count);
        if ((count/max_value) >= threshold){
          return true;

        }

        return false;
      };
};
