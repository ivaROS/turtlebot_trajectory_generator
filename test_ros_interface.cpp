/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <ros/ros.h>


#include <trajectory_generator_ros_interface.h>




namespace kobuki
{

/**
 * @ brief Sends a simple trajectory
 *
 * A simple program that sends a trajectory to the controller when a button is pressed.
 */
 
 

/* The rhs of x' = f(x) defined as a class */
class circle_traj_func : public traj_func{
    double vf_; //Forward vel
    double r_;  //radius of circle

public:
    circle_traj_func( double vf, double r) : vf_(vf), r_(r) { }
    
    void dState ( const state_type &x , state_type &dxdt , const double  t  )
    {
        
        dxdt[near_identity::XD_IND] = -vf_*sin((vf_/r_) * t);
        dxdt[near_identity::YD_IND] = vf_*cos((vf_/r_) * t);
    }
    
    
};
//]

 
 
 
 
class TrajectoryGeneratorBridgeTester 
{
public:
  TrajectoryGeneratorBridgeTester(ros::NodeHandle& nh, std::string& name) : nh_(nh), name_(name){
  };
  ~TrajectoryGeneratorBridgeTester(){};

  bool init()
  {

    trajectory_publisher_ = nh_.advertise< pips_trajectory_msgs::trajectory_points >("/desired_trajectory", 10);

    return true;
  };
  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
 
    pips_trajectory_msgs::trajectory_points generate_trajectory();

private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Publisher trajectory_publisher_;
  TrajectoryGeneratorBridge traj_gen_bridge;


};



pips_trajectory_msgs::trajectory_points TrajectoryGeneratorBridgeTester::generate_trajectory()
{
    std::string r_key, fw_vel_key;
    double fw_vel = .05;
    double r = .5;


    if(ros::param::search("r", r_key))
    {
        ros::param::get(r_key, r); 
    }
    
    if(ros::param::search("fw_vel", fw_vel_key))
    {
        ros::param::get(fw_vel_key, fw_vel); 
    }
    
    ni_trajectory_ptr traj = std::make_shared<ni_trajectory>();
    traj->header.frame_id = "base_link";
    traj->header.stamp = ros::Time::now();
    traj->trajpntr =  std::make_shared<circle_traj_func>(fw_vel,r);;
    traj->x0_ = traj_gen_bridge.initState();
      
    traj_gen_bridge.generate_trajectory(traj);
    
    traj->print();

    pips_trajectory_msgs::trajectory_points trajectory_msg = traj->toTrajectoryMsg ();
    trajectory_publisher_.publish(trajectory_msg);
    
    return trajectory_msg;
}



} // namespace kobuki
// %EndTag(FULLTEXT)%



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ros_interface");
    ros::NodeHandle nh;
    std::string name = ros::this_node::getName();
    kobuki::TrajectoryGeneratorBridgeTester tester(nh,name);
    


    if (tester.init())
    {
        tester.generate_trajectory();
        ros::spin();
    }
    else
    {
        ROS_ERROR_STREAM("Couldn't initialise test_ros_interface!");
    }

    ROS_INFO_STREAM("Program exiting");
    return 0;

}

