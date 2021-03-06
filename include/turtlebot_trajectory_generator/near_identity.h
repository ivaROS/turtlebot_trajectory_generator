
#ifndef NEAR_IDENTITY_H
#define NEAR_IDENTITY_H

#include <traj_generator.h>
#include <Eigen/Eigen>

//Since 'near identity' is no longer at the bottom but rather added from the top, it's probably ok for these dependencies to be included.
//It may be cleaner to do the conversion elsewhere, but this should be more straightforward
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pips_trajectory_msgs/trajectory_point.h>
#include <pips_trajectory_msgs/trajectory_points.h>
#include <tf/transform_datatypes.h>

namespace turtlebot_trajectory_generator
{

class ni_state : public trajectory_generator::TrajectoryState<ni_state,8>
{
public:
  enum STATE_INDICIES { X_IND=0, Y_IND=1, THETA_IND=2, V_IND=3, W_IND=4, LAMBDA_IND=5, XD_IND=6, YD_IND=7 };
  
  typedef pips_trajectory_msgs::trajectory_points trajectory_msg_t;
  
  //decltype(data)
  trajectory_generator::VectorReference<array> x,y,theta,v,w,lambda,xd,yd;
  
  ni_state() :
    x(data,ni_state::X_IND),
    y(data,ni_state::Y_IND),
    theta(data,ni_state::THETA_IND),
    v(data,ni_state::V_IND),
    w(data,ni_state::W_IND),
    lambda(data,ni_state::LAMBDA_IND),
    xd(data,ni_state::XD_IND),
    yd(data,ni_state::YD_IND)
  {
    lambda = .1;
  }
    
  ni_state(const ni_state& state) : 
    TrajectoryState(state),
    x(data,ni_state::X_IND),
    y(data,ni_state::Y_IND),
    theta(data,ni_state::THETA_IND),
    v(data,ni_state::V_IND),
    w(data,ni_state::W_IND),
    lambda(data,ni_state::LAMBDA_IND),
    xd(data,ni_state::XD_IND),
    yd(data,ni_state::YD_IND)
  {
//    lambda = .1; //shouldn't overwrite existing lambda value
  }
  
  template <typename T>
  ni_state(const T& state) : 
    TrajectoryState(),
    x(data,ni_state::X_IND),
    y(data,ni_state::Y_IND),
    theta(data,ni_state::THETA_IND),
    v(data,ni_state::V_IND),
    w(data,ni_state::W_IND),
    lambda(data,ni_state::LAMBDA_IND),
    xd(data,ni_state::XD_IND),
    yd(data,ni_state::YD_IND)
  {
    from(state);
    lambda = .1;
  }
  
  void operator=(const ni_state& state)
  {
    data = state.data;
  }
  
  bool isValid()
  {
    return !(lambda ==0);
  }
  
  
  void from(const geometry_msgs::Point& point)
  {
    x = point.x;
    y = point.y;
  }
  
  void from(const geometry_msgs::Quaternion& quaternion)
  {
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(quaternion, quat);
    
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    theta = yaw;
  }
  
  void from(const geometry_msgs::Twist& twist)
  {
    v = twist.linear.x;
    w = twist.angular.z;
  }
  
  //Note: the following conversions are generic and could probably be moved higher,
  //but would require adding some (slight) dependencies to trajectory_generator.h
  //Ideally, this functionality would be added in a different file that could be
  void from(const geometry_msgs::Pose& pose)
  {
    from(pose.position);
    from(pose.orientation);
  }
  
  void from(const nav_msgs::Odometry& odom)
  {
    //from(odom.pose.pose);
    from(odom.twist.twist);
  }
  
  //Implement any and all conversion methods
  void to(geometry_msgs::Point& point)
  {
    point.x = x;
    point.y = y;
    point.z = 0;
  }
  
  void to(geometry_msgs::Quaternion& quat)
  {
    double yaw = theta;
    double roll = 0;
    double pitch = 0;
    quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  }
  
  //if this function isn't implemented, falls back on 'to(geometry_msgs::Point& point)'
  void to(trajectory_generator::Desired<geometry_msgs::Point>& point)
  {
    geometry_msgs::Point& pt=point;
    pt.x=xd;
    pt.y=yd;
    pt.z=0;
  }
  
  pips_trajectory_msgs::trajectory_point toMsg()
  {
    pips_trajectory_msgs::trajectory_point point;
    //point.time = ros::Duration(times[i]);
    point.x = x;
    point.y = y;
    point.theta = theta;
    point.v = v;
    point.w = w;
    return point;
  }
};
  
struct ni_params
{
  ni_params() {}
  ni_params( double c_p, double c_d, double c_lambda, double epsilon ) : c_p_(c_p), c_d_(c_d), c_lambda_(c_lambda), epsilon_(epsilon) { }
  ni_params( double c_p, double c_d, double c_lambda, double epsilon, double v_max, double w_max, double a_max, double w_dot_max ) : c_p_(c_p), c_d_(c_d), c_lambda_(c_lambda), epsilon_(epsilon), v_max_(v_max), w_max_(w_max), a_max_(a_max), w_dot_max_(w_dot_max) { }
  
  double c_p_;
  double c_d_;
  double c_lambda_;
  double epsilon_;
  
  double v_max_ = std::numeric_limits<double>::infinity();
  double w_max_ = std::numeric_limits<double>::infinity();
  double a_max_ = std::numeric_limits<double>::infinity();
  double w_dot_max_ = std::numeric_limits<double>::infinity();
  
  bool verify()
  {
    //TODO: Add ROS_ASSERTs for each of these to enable explicit errors about these at run time and not in Release builds
    return c_p_>=0 && c_d_>=0 && c_lambda_>=0 && epsilon_>0
            && v_max_>0 && w_max_>0 && a_max_>0 && w_dot_max_>0;
  }
};

class near_identity {

    ni_params params_;

//     double c_p_;
//     double c_d_;
//     double c_lambda_;
//     double epsilon_;
//     
//     double v_max_ = std::numeric_limits<double>::infinity();
//     double w_max_ = std::numeric_limits<double>::infinity();
//     double a_max_ = std::numeric_limits<double>::infinity();
//     double w_dot_max_ = std::numeric_limits<double>::infinity();

public:

    
    near_identity( double c_p, double c_d, double c_lambda, double epsilon ) : params_(c_p, c_d, c_lambda, epsilon) { }

    near_identity( double c_p, double c_d, double c_lambda, double epsilon, double v_max, double w_max, double a_max, double w_dot_max ) : params_(c_p, c_d, c_lambda, epsilon, v_max, w_max, a_max, w_dot_max) { }
    
    near_identity( ni_params params) : params_(params) {}

    void operator() ( const ni_state &state , ni_state &state_dot, const double /* t*/  )
    {
        //load state variables
        double x = state[ni_state::X_IND];
        double y = state[ni_state::Y_IND];
        double theta = state[ni_state::THETA_IND];
        double v = state[ni_state::V_IND];
        double w = state[ni_state::W_IND];
        double lambda = state[ni_state::LAMBDA_IND];
        double x_d = state[ni_state::XD_IND];
        double y_d = state[ni_state::YD_IND];
        double x_d_dot = state_dot[ni_state::XD_IND];
        double y_d_dot = state_dot[ni_state::YD_IND];

        Eigen::Matrix2d R;
        R << cos(theta), -sin(theta), 
             sin(theta), cos(theta);
             
        double lambda_dot = -params_.c_lambda_*(lambda - params_.epsilon_);
        

        //Now find derivatives of state variables
        //x_dot = (R*e1)*v1;
        double x_dot = R(0,0)*v;
        double y_dot = R(1,0)*v;
        double theta_dot = w;
        
        //Now find tau
        Eigen::Vector2d tau = getTau(x,y,theta,v,w,lambda,x_d,y_d,x_d_dot,y_d_dot,R,lambda_dot);

        double v_dot = limitV(tau, v);
        double w_dot = limitW(tau, w);


        state_dot[ni_state::X_IND] = x_dot;
        state_dot[ni_state::Y_IND] = y_dot;
        state_dot[ni_state::THETA_IND] = theta_dot;
        state_dot[ni_state::V_IND] = v_dot;
        state_dot[ni_state::W_IND] = w_dot;
        state_dot[ni_state::LAMBDA_IND] = lambda_dot;
        //std::vector<double> state_dot = {x_dot, y_dot, theta_dot, v_dot, w_dot, lambda_dot};

    }
    
private:
    
    inline
    Eigen::Vector2d getTau( double x, double y, double theta, double v, double w, double lambda, double x_d, double y_d, double x_d_dot, double y_d_dot )
    {
    
        Eigen::Matrix2d R;
        R << cos(theta), -sin(theta), 
             sin(theta), cos(theta);
             
        double lambda_dot = -params_.c_lambda_*(lambda - params_.epsilon_);
        return getTau(x,y,theta,v,w,lambda,x_d,y_d,x_d_dot,y_d_dot,R,lambda_dot);
    }
    
    
    inline
    __attribute__((always_inline))
    Eigen::Vector2d getTau( double x, double y, double theta, double v, double w, double lambda, double x_d, double y_d, double x_d_dot, double y_d_dot, const Eigen::Matrix2d& R, double lambda_dot )
    {
        Eigen::Vector2d xy;
        xy << x,
              y;
                           
        Eigen::Vector2d vw;
        vw << v,
              w;

        Eigen::Vector2d xy_d;
        xy_d << x_d, 
                y_d;
                
        Eigen::Vector2d xy_d_dot;
        xy_d_dot << x_d_dot,
                    y_d_dot;

        //R_lambda = [R*e1 lambda*R*e2];
        Eigen::Matrix2d R_lambda;
        R_lambda << R(0,0), lambda*R(0,1), 
                    R(1,0), lambda*R(1,1);

        //R_lambda_inv = [R*e1 R*e2/lambda]';
        Eigen::Matrix2d R_lambda_inv;
        R_lambda_inv << R(0,0),        R(1,0), 
                        R(0,1)/lambda, R(1,1)/lambda;

        Eigen::Matrix2d w_hat;
        w_hat << 0,             -lambda*w, 
                (1.0/lambda)*w, lambda_dot/lambda;

        //q = xy + lambda*R*e1;
    /*    Eigen::Vector2d q(x + lambda*R(0,0), 
                          y + lambda*R(1,0)); */
        Eigen::Vector2d q = xy + lambda*R.col(0);     
                          
     //   std::cout << "q: " << q(0) << ", " << q(1) << std::endl;
        
        //p = R_lambda*v + lambda_dot*R_lambda*e1;
     /*   Eigen::Vector2d p(v + lambda_dot*R_lambda(0,0), 
                          w + lambda_dot*R_lambda(1,0));*/

        Eigen::Vector2d p = R_lambda*vw + lambda_dot*R_lambda.col(0);

     //   std::cout << "p: " << p(0) << ", " << p(1) << std::endl;
        
        
        //u can be any expression that will feedback stabilize a linear system
        //here, using 2nd order feedback controller   ; + x_d_dot_dot
        Eigen::Vector2d u = -params_.c_p_*(q - xy_d) - params_.c_d_*(p - xy_d_dot);

   //     std::cout << "u: " << u(0) << ", " << u(1) << std::endl;

        //Now find tau
        Eigen::Vector2d tau = R_lambda_inv*u - w_hat*vw - lambda_dot*(w_hat - params_.c_lambda_*Eigen::Matrix2d::Identity())*Eigen::Matrix<double, 2, 1>::Identity();
        
        return tau;
   
    }
    

    /* Generic saturation function for variable X. */
    inline
    static double saturate(double X, double minX, double maxX)
    {
        if(X >= maxX)
        {
            return maxX;
        }
        else if(X <= minX)
        {
            return minX;
        }
        else
        {
            return X;
        }
    }
    
    /* Generic saturation function for variable X_dot given limits on X_dot and X. */
    inline
    static double applyLimits(double X_dot, double X, double minX, double maxX, double minX_dot, double maxX_dot)
    {
        if(X >= maxX)
        {
            return saturate(X_dot, minX_dot, 0);
        }
        else if(X <= minX)
        {
            return saturate(X_dot, 0, maxX_dot);
        }
        else
        {
            return saturate(X_dot, minX_dot, maxX_dot);
        }   
    }
    
    inline
    double limitV(const Eigen::Vector2d& tau, double v)
    {
        return applyLimits(tau[0], v, -params_.v_max_, params_.v_max_, -params_.a_max_, params_.a_max_);
    }
    
    inline
    double limitW(const Eigen::Vector2d& tau, double w)
    {
        return applyLimits(tau[1], w, -params_.w_max_, params_.w_max_, -params_.w_dot_max_, params_.w_dot_max_);
    }
};




//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class desired_traj_func {
  
public:
  //TODO: was there ever a reason to have this?
  virtual void init ( const ni_state &x0 )
  {
    //std::cout << "This should only print if an init function is not defined" << std::endl;
  }
  
  virtual void dState ( const ni_state &x , ni_state &dxdt , const double  t  )=0;
  
  
  typedef std::shared_ptr<desired_traj_func> Ptr;
  
};
//]



//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class ni_controller : public trajectory_generator::traj_func<ni_controller, ni_state>
{
  
  near_identity ni_;
  desired_traj_func::Ptr traj_;   //Will look into using reference to function 
  
  
public:
  ni_controller( near_identity ni) :  ni_(ni) { }
  
  void setTrajFunc(const desired_traj_func::Ptr& traj)
  {
    traj_ = traj;
  }
  
  
  
  void operator_impl ( const ni_state &x , ni_state &dxdt , const double  t  )
  {
    traj_->dState(x,dxdt,t);
    ni_(x,dxdt,t);
  }
  
  typedef std::shared_ptr<ni_controller> Ptr;
  
};

typedef ni_state state_type;
typedef ni_controller traj_func_type;
typedef trajectory_generator::traj_generator<state_type, traj_func_type> TrajectoryGenerator;
typedef turtlebot_trajectory_generator::desired_traj_func::Ptr des_traj_func_ptr;


} //namespace turtlebot_trajectory_generator

#endif  /* !near_identity.h sen */

