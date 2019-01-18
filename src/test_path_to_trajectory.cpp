#include <boost/math/interpolators/cubic_b_spline.hpp>


/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <ros/ros.h>

#include <turtlebot_trajectory_generator/near_identity.h>
#include <trajectory_generator_ros_interface.h>

#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision



/* The rhs of x' = f(x) defined as a class */
class path_traj_func : public virtual desired_traj_func{
  
  
public:
  path_traj_func(const nav_msgs::PathConstPtr path)
  {     
    if(path)
    {
      double grid_spacing = .05;
      double v_des=.3;
      double step = grid_spacing / v_des;
      std::vector<double>x,y;
      
      tf=step*path->poses.size();
      
      for(const geometry_msgs::PoseStamped& pose : path->poses)
      {
        x.push_back(pose.pose.position.x);
        y.push_back(pose.pose.position.y);
      }
      spline_x = boost::math::cubic_b_spline<double>(x.begin(),x.end(),0,step);
      spline_y = boost::math::cubic_b_spline<double>(y.begin(),y.end(),0,step);
      
    }
  }
  
  //TODO: implement 'init' function and use state to provide values for left_endpoint derivatives
  
  void dState ( const ni_state &x , ni_state &dxdt , const double  t  )
  {
    dxdt.xd = spline_x.prime(t);
    dxdt.yd = spline_y.prime(t);
    //ROS_INFO_STREAM("dxdt: " << dxdt.xd << ", dydt: " << dxdt.yd);
  }
  
  double getTF()
  {
    return tf;
  }
  
private:
  boost::math::cubic_b_spline<double> spline_x, spline_y;
  double tf;
};
//]
 
 
 
class TrajectoryGeneratorBridgeTester 
{
public:
  typedef ni_state state_type;
  typedef ni_controller traj_func_type;
  typedef trajectory_states<state_type, traj_func_type> traj_type;
  typedef std::shared_ptr<traj_type> traj_type_ptr;
  
  
private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Publisher trajectory_publisher_, path_pub_;
  ros::Subscriber path_sub_;
  TrajectoryGeneratorBridge<state_type, traj_func_type>  traj_gen_bridge;
  
  
  
public:
  TrajectoryGeneratorBridgeTester(ros::NodeHandle& nh, std::string& name) : nh_(nh), name_(name){
  };
  ~TrajectoryGeneratorBridgeTester(){};
  

  /**
   * Set-up necessary publishers/subscribers
   * @return true, if successful
   */
  bool init()
  {
    
    trajectory_publisher_ = nh_.advertise< pips_trajectory_msgs::trajectory_points >("/desired_trajectory", 10);
    path_pub_ = nh_.advertise< nav_msgs::Path>("/desired_path", 10);
    
    path_sub_ = nh_.subscribe("/move_base/DWAPlannerROS/global_plan", 10, &TrajectoryGeneratorBridgeTester::generate_trajectory, this);
    return true;
    
  };

 
  
  
  void generate_trajectory(const nav_msgs::PathConstPtr& input_path_msg)
  {
    
    
    path_traj_func::Ptr dtraj = std::make_shared<path_traj_func>(input_path_msg);
    near_identity ni(100,100,100,.01);    
    traj_func_type::Ptr nc=std::make_shared<traj_func_type>(ni);
    nc->setTrajFunc(dtraj);
    
    double tf;
    double grid_spacing = .05;
    double v_des=.3;
    double step = grid_spacing / v_des;
    tf = step*input_path_msg->poses.size();
    
    ROS_INFO_STREAM("Tf=" << tf);
    
    traj_type_ptr traj = std::make_shared<traj_type>();
    traj->header.frame_id = "base_footprint";
    traj->header.stamp = ros::Time::now();
    traj->trajpntr = nc ;
    traj->params = std::make_shared<traj_params>();
    traj->params->tf=tf;
    traj->x0_.lambda=.3;
    traj->x0_.yd=0;
    traj->x0_.xd=0;
    
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
    
    
    pips_trajectory_msgs::trajectory_points trajectory_msg = traj->toMsg ();
    trajectory_publisher_.publish(trajectory_msg);
    
    nav_msgs::Path::ConstPtr path_msg = traj->toPathMsg();    
    path_pub_.publish(path_msg);
    
  }
  

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_ros_interface");
    ros::NodeHandle nh;
    std::string name = ros::this_node::getName();
    TrajectoryGeneratorBridgeTester tester(nh,name);
    
    ros::Duration t(2);
    t.sleep();

    ros::Duration d(150);
    if (tester.init())
    {
      ros::spin();
//       while(ros::ok())
//       {
//         //tester.generate_trajectory();
//         ros::spinOnce();
//         d.sleep();
//       }
    }
    else
    {
        ROS_ERROR_STREAM("Couldn't initialise test_ros_interface!");
    }

    ROS_INFO_STREAM("Program exiting");
    return 0;

}


