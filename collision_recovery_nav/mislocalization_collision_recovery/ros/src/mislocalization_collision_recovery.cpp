#include "mislocalization_collision_recovery/mislocalization_collision_recovery.h"
#include <pluginlib/class_list_macros.h>

// register this class as a recovery behavior plugin
PLUGINLIB_DECLARE_CLASS(mislocalization_collision_recovery, MisLocalizationCollisionRecovery,
                        mislocalization_collision_recovery::MisLocalizationCollisionRecovery,
                        fault_core::FaultRecoveryBehavior)

using namespace fault_core;
namespace mislocalization_collision_recovery
{

  MisLocalizationCollisionRecovery::MisLocalizationCollisionRecovery()
  {
    fault_cause_ = FaultTopology::MISLOCALIZATION;
    ros::NodeHandle n;
    client_ = n.serviceClient<std_srvs::Empty>("/global_localization");
    ROS_INFO("Constructor MisLocalizationCollisionRecovery");
  }


  MisLocalizationCollisionRecovery::~MisLocalizationCollisionRecovery()
  {

  }

  void MisLocalizationCollisionRecovery::initialize()
  {

  }

  void MisLocalizationCollisionRecovery::runFaultBehavior()
  {
    if (ros::service::waitForService ("/global_localization", 100)) {
      std_srvs::Empty s;
      if(!client_.call(s)) {
        ROS_ERROR ("Error calling service");
      }
    }
    else {
      ROS_ERROR ("Could not find service");
    }
  }

}  // namespace MisLocalization_collision_recovery
