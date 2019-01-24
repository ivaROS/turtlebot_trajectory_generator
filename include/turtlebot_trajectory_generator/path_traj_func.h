#ifndef PATH_TRAJ_FUNC_H
#define PATH_TRAJ_FUNC_H

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

#include <boost/math/interpolators/cubic_b_spline.hpp>
#include <turtlebot_trajectory_generator/near_identity.h>


namespace turtlebot_trajectory_generator
{


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
  
  /*
  void init(const ni_state &x)
  {
    x.xd=x[0];
    x.yd=y[0];
  }
  */
  
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
    double grid_spacing = .1;
    double v_des=.3;
    double step = grid_spacing / v_des;
    //std::vector<double>x,y;
    
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
  std::vector<double>x,y;
  
};

}

#endif //namespace turtlebot_trajectory_generator
