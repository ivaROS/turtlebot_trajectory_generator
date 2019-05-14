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

#include <turtlebot_trajectory_generator/near_identity.h>
#include <trajectory_generator_ros_interface.h>

#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision


namespace turtlebot_trajectory_generator
{

/**
 * @ brief Sends a simple trajectory
 *
 * A simple program that sends a trajectory to the controller when a button is pressed.
 */
 
 

/* The rhs of x' = f(x) defined as a class */
class circle_traj_func : public desired_traj_func{
    double vf_; //Forward vel
    double r_;  //radius of circle

public:
    circle_traj_func( double vf, double r) : vf_(vf), r_(r) { }
    
    void dState ( const ni_state &x , ni_state &dxdt , const double  t  )
    {
        
        dxdt.xd = -vf_*sin((vf_/r_) * t);
        dxdt.yd = vf_*cos((vf_/r_) * t);
    }
    
    
};
//]


/* The rhs of x' = f(x) defined as a class */
class serpentine_traj_func : public virtual desired_traj_func{
  
  double m_v;
  double m_amp;
  double m_f;
  
public:
  serpentine_traj_func( double v, double amp, double f ) : m_v(v), m_amp(amp), m_f(f) { }
  
  void dState ( const ni_state &x , ni_state &dxdt , const double  t  )
  {
    dxdt.xd = m_v;
    dxdt.yd = sin(t*2.0*3.14*m_f) * m_amp;
  }
};
//]
 
 
 
class TrajectoryGeneratorBridgeTester 
{
public:
  typedef ni_state state_type;
  typedef ni_controller traj_func_type;
  typedef trajectory_generator::trajectory_states<state_type, traj_func_type> traj_type;
  typedef std::shared_ptr<traj_type> traj_type_ptr;
  
  
private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Publisher trajectory_publisher_, path_pub_;
  trajectory_generator::TrajectoryGeneratorBridge<state_type, traj_func_type>  traj_gen_bridge;
  
  
  
public:
  TrajectoryGeneratorBridgeTester(ros::NodeHandle& nh, std::string& name) : nh_(nh), name_(name){
  };
  ~TrajectoryGeneratorBridgeTester(){};
  


  bool init()
  {
    trajectory_publisher_ = nh_.advertise< pips_trajectory_msgs::trajectory_points >("/desired_trajectory", 10);
    path_pub_ = nh_.advertise< nav_msgs::Path>("/desired_path", 10);
    return true;
    
  };
  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
 
  
  
  pips_trajectory_msgs::trajectory_points generate_trajectory()
  {
    std::string r_key, fw_vel_key;
    double fw_vel = .25;
    double r = 5;
    
    //desired_traj_func::Ptr dtraj = std::make_shared<serpentine_traj_func>(.2,0.15,.2);
    desired_traj_func::Ptr dtraj = std::make_shared<circle_traj_func>(fw_vel,r);
    near_identity ni(100,100,100,.01);    
    traj_func_type::Ptr nc=std::make_shared<traj_func_type>(ni);
    nc->setTrajFunc(dtraj);
    
    
    traj_type_ptr traj = std::make_shared<traj_type>();
    traj->header.frame_id = "base_footprint";
    traj->header.stamp = ros::Time::now();
    traj->trajpntr = nc ;
    traj->params = std::make_shared<trajectory_generator::traj_params>();
    traj->params->tf=150;
    traj->x0_[ni_state::LAMBDA_IND]=.3;
    traj->x0_.yd=1;
    traj->x0_.xd=1;
    
    traj_gen_bridge.generate_trajectory(traj);
    
    ROS_INFO_STREAM("Size: " << traj->x_vec.size());
    
    
    std::cout << "Time" << '\t' << "Error" << '\t' << 'x' << '\t' << 'y' << '\t' << "theta" << '\t' << 'v' << '\t' << 'w' << '\t' << "lambda" << '\t' << "xd" << '\t' << "yd" << std::endl;
    
    for( size_t i=0; i < traj->num_states(); i++ )
    {
      state_type& state = traj->x_vec[i];
      
      double error_x = state.x-state.xd;
      double error_y = state.y-state.yd;
      
      double error = sqrt(error_x*error_x + error_y*error_y);
      //NOTE: This doesn't work, for some reason
      std::cout << std::fixed << std::setw(4) <<std::setprecision(4) << traj->times[i] << "\t" << error << "\t" << state.x << "\t" << state.y << "\t" << state.theta << "\t" << state.v << "\t" << state.w << "\t" << state.lambda << "\t" << state.xd << "\t" << state.yd << std::endl;
    }
    
 
  for( size_t i=0; i < traj->num_states(); i++ )
  {
    double error_x = traj->x_vec[i][ni_state::X_IND] - traj->x_vec[i][ni_state::XD_IND];
    double error_y = traj->x_vec[i][ni_state::Y_IND] - traj->x_vec[i][ni_state::YD_IND];
    
    double error = sqrt(error_x*error_x + error_y*error_y);
    printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", traj->times[i], error, traj->x_vec[i][0],traj-> x_vec[i][1], traj->x_vec[i][2], traj->x_vec[i][3], traj->x_vec[i][4], traj->x_vec[i][5],traj->x_vec[i][6], traj->x_vec[i][7]);
  }
    
    
    pips_trajectory_msgs::trajectory_points trajectory_msg = traj->toMsg ();
    trajectory_publisher_.publish(trajectory_msg);
    
    nav_msgs::Path::ConstPtr path_msg = traj->toPathMsg();    
    path_pub_.publish(path_msg);
    
    return trajectory_msg;
  }
  

};




} // namespace turtlebot_trajectory_generator
// %EndTag(FULLTEXT)%



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ros_interface");
    ros::NodeHandle nh;
    std::string name = ros::this_node::getName();
    turtlebot_trajectory_generator::TrajectoryGeneratorBridgeTester tester(nh,name);
    
    ros::Duration t(2);
    t.sleep();

    ros::Duration d(150);
    if (tester.init())
    {
      while(ros::ok())
      {
        tester.generate_trajectory();
        ros::spinOnce();
        d.sleep();
      }
    }
    else
    {
        ROS_ERROR_STREAM("Couldn't initialise test_ros_interface!");
    }

    ROS_INFO_STREAM("Program exiting");
    return 0;

}


