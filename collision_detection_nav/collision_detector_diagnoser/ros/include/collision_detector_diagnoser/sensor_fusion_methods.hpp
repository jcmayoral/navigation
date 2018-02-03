#include <list>
#include <ros/ros.h>
#include <fusion_msgs/sensorFusionMsg.h>
#include <std_msgs/Float32.h>

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
        if (counter >= v.size()*threshold){
          ROS_WARN_STREAM("Consensus Collision Detected");
          ROS_DEBUG_STREAM("Counter " << counter);
          return true;
        }

        return false;
      };
};

class WeightedApproach : public SensorFusionApproach {
    public:
      bool detect(list<fusion_msgs::sensorFusionMsg> v){
        ROS_DEBUG("Weighted");
        double max_value = 0; //TODO
        double count = 0;

        for (std::list<fusion_msgs::sensorFusionMsg>::iterator it=v.begin(); it != v.end(); ++it){
          max_value+= fusion_msgs::sensorFusionMsg::ERROR * it->weight;
          count += it->msg * it->weight;
        }

        ROS_DEBUG_STREAM("Count " << count);
        if ((count/max_value) >= threshold){
          ROS_WARN_STREAM("Weighted Collision Detected: " << count << " Of " << max_value << " Percentage:" << (count/max_value));
          return true;

        }

        return false;
      };
};

class KalmanFilterApproach : public SensorFusionApproach {
    public:

      bool detect(list<fusion_msgs::sensorFusionMsg> v){
        cout << "hglglg";
        ROS_INFO("Kalman Filter");
        // Based on http://www.ece.montana.edu/seniordesign/archive/SP14/UnderwaterNavigation/kalman_filter.html
        std::vector<float> z(v.size());
        for (std::list<fusion_msgs::sensorFusionMsg>::iterator it=v.begin(); it != v.end(); ++it){
          for(int i = 0; i < it->data.size(); i++){
            z[i] = it->data[i]; // initialize x (measured)
            p[i] = it->data[i]; // TODO this part just simplifies covariance matrix but it must be calculated.
                                // one variance for all sensors or i*i covariance
          }
        }

        x.clear();
        y.clear();
        k.clear();

        for(int i = 0; i < z.size(); i++){
          x.push_back(threshold);
          y.push_back(z[i] - p[i]); // innovation
          k.push_back(1);
        }


        for (auto i = y.begin(); i != y.end(); ++i){
          std::cout << *i << ' ';
        }

        s = p; // H = 1


        for(int i = 0; i < x.size(); i++){
          x[i] = x[i] + k[i]*y[i]; // assuming error should be constant x_p = A x(n-1) + B u(n)
        }

        for (auto j = k.begin(); j != k.end(); ++j){
          if (*j > threshold){
            return true;
          }
        }

        return false;
      };

    private:
      std::vector<float> x;//state prediction
      std::vector<float> y;//innovation
      std::vector<float> k;//filter gain
      std::vector<float> p;//covariance prediction
      std::vector<float> s;// innovation covariance
      std::vector<float> q;//noise TODO 0 for now

};
