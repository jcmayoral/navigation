#include "simple_recovery_behavior/simple_recovery_behavior.hpp"
#include <pluginlib/class_list_macros.h>

// register this class as a recovery behavior plugin
PLUGINLIB_DECLARE_CLASS(simple_recovery_behavior, SimpleRecoveryBehavior,
                        simple_recovery_behavior::SimpleRecoveryBehavior,
                        nav_core::RecoveryBehavior)

namespace simple_recovery_behavior
{

  SimpleRecoveryBehavior::SimpleRecoveryBehavior()
  {
    ROS_INFO("Constructor SimpleRecoveryBehavior");
  }


  SimpleRecoveryBehavior::~SimpleRecoveryBehavior()
  {

  }


  void SimpleRecoveryBehavior::initialize(std::string name,
    tf::TransformListener* tfListener,
    costmap_2d::Costmap2DROS* globalCostmapROS,
    costmap_2d::Costmap2DROS* localCostmapROS)
  {

  }

  void SimpleRecoveryBehavior::runBehavior()
  {
  }

}  // namespace simple_recovery_behavor
