#ifndef TURTLEBOT_TRAJECTORY_GENERATOR_ROS_INTERFACE_H
#define TURTLEBOT_TRAJECTORY_GENERATOR_ROS_INTERFACE_H

#include <turtlebot_trajectory_generator/near_identity.h>
#include <trajectory_generator_ros_interface.h>

namespace turtlebot_trajectory_generator
{
  typedef trajectory_generator::trajectory_states<state_type, traj_func_type> traj_type;
  typedef std::shared_ptr<traj_type> traj_type_ptr;

  typedef trajectory_generator::TrajectoryGeneratorBridge<state_type, traj_func_type>  traj_gen_bridge;




}


#endif //TURTLEBOT_TRAJECTORY_GENERATOR_ROS_INTERFACE_H
