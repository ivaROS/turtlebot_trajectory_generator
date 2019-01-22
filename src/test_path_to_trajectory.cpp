#include <boost/math/interpolators/cubic_b_spline.hpp>


/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <ros/ros.h>

#include <turtlebot_trajectory_generator/near_identity.h>
#include <trajectory_generator_ros_interface.h>

#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

#include <tf2/convert.h>
#include <turtlebot_trajectory_generator/tf2_path_msg.h>

#include <iostream>     // std::cout, std::fixed
#include <iomanip>      // std::setprecision

#include <tf_utils/odom_to_tf.h>

/* The rhs of x' = f(x) defined as a class */
class path_traj_func : public virtual desired_traj_func{
  
  
public:
  path_traj_func(const nav_msgs::PathConstPtr& path)
  {     
    if(path)
    {
      construct(path->poses);
    }
  }
  
  path_traj_func(const nav_msgs::Path& path)
  {     
    construct(path.poses);
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
  void construct(const std::vector<geometry_msgs::PoseStamped>& poses)
  {
    double grid_spacing = .05;
    double v_des=.3;
    double step = grid_spacing / v_des;
    std::vector<double>x,y;
    
    tf=poses.size();
    
    //TODO: use custom iterator to avoid this copy
    for(const geometry_msgs::PoseStamped& pose : poses)
    {
      x.push_back(pose.pose.position.x);
      y.push_back(pose.pose.position.y);
    }
    spline_x = boost::math::cubic_b_spline<double>(x.begin(),x.end(),0,step,0,0); //Assuming start from rest and end at rest
    spline_y = boost::math::cubic_b_spline<double>(y.begin(),y.end(),0,step,0,0);
    
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
  
  typedef tf2_ros::MessageFilter<nav_msgs::Path> PathFilter;
  typedef std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr;
  typedef std::shared_ptr<tf2_ros::TransformListener> transform_listener_ptr;
  
private:
  ros::NodeHandle nh_;
  std::string name_;
  ros::Publisher trajectory_publisher_, path_pub_, transformed_path_pub_;
  ros::Subscriber odom_sub_;
  TrajectoryGeneratorBridge<state_type, traj_func_type>  traj_gen_bridge;
  
  tf_buffer_ptr tfBuffer_;
  transform_listener_ptr tf_listener_;
  

  message_filters::Subscriber<nav_msgs::Path> path_sub_;
  std::shared_ptr<PathFilter> path_filter_;
  
  nav_msgs::OdometryConstPtr cur_odom_;
  
public:
  TrajectoryGeneratorBridgeTester(ros::NodeHandle& nh, std::string& name) : nh_(nh), name_(name){
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(); //optional parameter: ros::Duration(cache time) (default=10) (though it doesn't seem to accept it!)
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    
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
    transformed_path_pub_ = nh_.advertise<nav_msgs::Path>("/transformed_input_path",10);
    path_filter_.reset(new PathFilter(path_sub_, *tfBuffer_, "odom", 5, nh_));
    path_sub_.subscribe(nh_, "/move_base/NavfnROS/plan", 5);
    
    path_filter_->registerCallback(boost::bind(&TrajectoryGeneratorBridgeTester::generate_trajectory, this, _1));
    path_filter_->setTolerance(ros::Duration(0.01));
    
    //path_sub_ = nh_.subscribe("/move_base/NavfnROS/plan", 10, &TrajectoryGeneratorBridgeTester::generate_trajectory, this);
    odom_sub_ = nh_.subscribe("/odom", 10, &TrajectoryGeneratorBridgeTester::odometryCB, this);
    
    return true;
    
  };

 
  void odometryCB(const nav_msgs::OdometryConstPtr& odom_msg)
  {
    cur_odom_ = odom_msg;
    tf_utils::AddToBuffer(odom_msg, tfBuffer_, "odometry_msg");
  }
  
  void generate_trajectory(const nav_msgs::PathConstPtr& input_path_msg)
  {
    if(!input_path_msg)
    {
      ROS_WARN_STREAM("Input path_msg is null!");
      return;
    }
    else if(input_path_msg->poses.size()<3)
    {
      ROS_WARN_STREAM("Input path_msg has only " << input_path_msg->poses.size() << " poses; at least 3 are required for interpolation");
      return;
    }
    
    if(cur_odom_==nullptr)
    {
      ROS_WARN_STREAM("No current odom!");
      return;
    }
    
    const nav_msgs::OdometryConstPtr odom = cur_odom_;
    
    //Ensure that path is in odometry frame_id
    nav_msgs::Path path_t;
    
    try
    {
      path_t = tfBuffer_->transform(*input_path_msg, odom->header.frame_id, odom->header.stamp,"map");
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN_STREAM("Error transforming path: from [" << input_path_msg->header.frame_id << "](" << input_path_msg->header.stamp << ") to " << odom->header.frame_id << "](" << odom->header.stamp << "): " << ex.what());
      return;
    }
    
    transformed_path_pub_.publish(path_t);
/*    
    double px, py, pth;
    vector<geometry_msgs::PoseStamped> odom_path;
    
    for ( unsigned int i = 0; i < num_poses_to_check; ++i ) {
      traj.getPoint ( i, px, py, pth );
      //NOTE: With the current implementation, there's no advantage to using stamped poses
      
      
      tf2::doTransform(pose,transformed_pose,global_local_transform_);
      
      //         pose.pose.orientation.x = 1.0;
      //         pose.pose.orientation.y = 0.0;
      //         pose.pose.orientation.z = 0.0;
      //         pose.pose.orientation.w = 0.0;
      odom_path.push_back ( transformed_pose );
    }
    */
    
    path_traj_func::Ptr dtraj = std::make_shared<path_traj_func>(path_t);
    
    double v_max=.5;
    double w_max=4;
    double a_max=.55;
    double w_dot_max=1.78;
    
    near_identity ni(1,5,1,.01,v_max,w_max,a_max,w_dot_max);    
    traj_func_type::Ptr nc=std::make_shared<traj_func_type>(ni);
    nc->setTrajFunc(dtraj);
    
    double tf;
    double grid_spacing = .05;
    double v_des=.3;
    double step = grid_spacing / v_des;
    tf = step*input_path_msg->poses.size();
    
    ROS_INFO_STREAM("Tf=" << tf);
    
    traj_type_ptr traj = std::make_shared<traj_type>();
    //traj->header.frame_id = "base_footprint";
    //traj->header.stamp = ros::Time::now();
    traj->header = path_t.header;
    traj->trajpntr = nc ;
    traj->params = std::make_shared<traj_params>();
    traj->params->tf=tf;
    traj->x0_.from(odom->pose.pose);
    traj->x0_.from(odom->twist.twist);
    traj->x0_.lambda=.3;
    traj->x0_.xd=path_t.poses[0].pose.position.x;
    traj->x0_.yd=path_t.poses[0].pose.position.y;
    
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


