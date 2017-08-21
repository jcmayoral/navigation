#include "simple_collision_recovery/simple_collision_recovery.h"
#include <pluginlib/class_list_macros.h>

// register this class as a recovery behavior plugin
PLUGINLIB_DECLARE_CLASS(simple_collision_recovery, SimpleCollisionRecovery,
                        simple_collision_recovery::SimpleCollisionRecovery,
                        fault_core::FaultRecoveryBehavior)

using namespace fault_core;
namespace simple_collision_recovery
{

  SimpleCollisionRecovery::SimpleCollisionRecovery()
  {
    fault_cause_ = FaultTopology::UNKNOWN;
    ROS_INFO("Constructor SimpleCollisionRecovery");
  }


  SimpleCollisionRecovery::~SimpleCollisionRecovery()
  {

  }

  void SimpleCollisionRecovery::initialize(std::string name,
    tf::TransformListener* tfListener,
    costmap_2d::Costmap2DROS* globalCostmapROS,
    costmap_2d::Costmap2DROS* localCostmapROS)
  {

  }

  void SimpleCollisionRecovery::runFaultBehavior()
  {
  }

}  // namespace simple_collision_recovery
