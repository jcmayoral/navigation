# Move_base_fault_tolerant navigation stack

This navigation stack is an extension of the popular move_base package. This extension provides a fault detection and recovery architecture to handle unexpected collisions during motion execution improving the default stack with fault tolerant skills. The current solution contemplates the collision detection through sensor fusion algorithms by the merge of different sensing sources.

Move_base_fault_tolerant provides two additional plugin types: FaultDetector and FaultRecovery. Meaning that the diversity of possible detected faults and their recovery is not limited by the approaches given in the current repository. It is important to mention that a FaultRecovery plugin is attached to one particular fault type, in other words, in difference to the classical recovery_behaviors, the execution of the fault_recovery_behaviors is not sequential then a particular recovery method will be executed without any need of executed other ones once a fault is detected.

This stack consists of:

1. fault_core: custom messages for collisions.
1. fusion_msgs: custom messges for sensor_fusion approach.
1. navigation_manager: consists in a modified version of the move_base(most of them setters and getter to private attributes). In addition the move_base_fault_tolerant packages inherits the modified version to mantain an strategy to minimize commits.
1. collision_detection_nav: provides FaultDetection plugins packages.
1. collision_recovery_nav: provides FaultRecovery plugin packages.
1. collisions_launch: contains launch files to run the move_base_fault_recovery
