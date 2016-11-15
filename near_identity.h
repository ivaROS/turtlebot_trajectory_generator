#include <Eigen/Eigen>

#ifndef NEAR_IDENTITY_H
#define NEAR_IDENTITY_H

typedef std::vector< double > state_type;

class near_identity {



    double c_p_;
    double c_d_;
    double c_lambda_;
    double epsilon_;
    
    double v_max_ = std::numeric_limits<double>::infinity();
    double w_max_ = std::numeric_limits<double>::infinity();
    double a_max_ = std::numeric_limits<double>::infinity();
    double w_dot_max_ = std::numeric_limits<double>::infinity();

public:

    enum STATE_INDICIES { X_IND=0, Y_IND=1, THETA_IND=2, V_IND=3, W_IND=4, LAMBDA_IND=5, XD_IND=6, YD_IND=7 };
    
    near_identity( double c_p, double c_d, double c_lambda, double epsilon ) : c_p_(c_p), c_d_(c_d), c_lambda_(c_lambda), epsilon_(epsilon) { }

    near_identity( double c_p, double c_d, double c_lambda, double epsilon, double v_max, double w_max, double a_max, double w_dot_max ) : c_p_(c_p), c_d_(c_d), c_lambda_(c_lambda), epsilon_(epsilon), v_max_(v_max), w_max_(w_max), a_max_(a_max), w_dot_max_(w_dot_max) { }

    void operator() ( const state_type &state , state_type &state_dot, const double /* t*/  )
    {
        //load state variables
        double x = state[near_identity::X_IND];
        double y = state[near_identity::Y_IND];
        double theta = state[near_identity::THETA_IND];
        double v = state[near_identity::V_IND];
        double w = state[near_identity::W_IND];
        double lambda = state[near_identity::LAMBDA_IND];
        double x_d = state[near_identity::XD_IND];
        double y_d = state[near_identity::YD_IND];
        double x_d_dot = state_dot[near_identity::XD_IND];
        double y_d_dot = state_dot[near_identity::YD_IND];

        Eigen::Matrix2d R;
        R << cos(theta), -sin(theta), 
             sin(theta), cos(theta);
             
        double lambda_dot = -c_lambda_*(lambda - epsilon_);
        

        //Now find derivatives of state variables
        //x_dot = (R*e1)*v1;
        double x_dot = R(0,0)*v;
        double y_dot = R(1,0)*v;
        double theta_dot = w;
        
        //Now find tau
        Eigen::Vector2d tau = getTau(x,y,theta,v,w,lambda,x_d,y_d,x_d_dot,y_d_dot,R,lambda_dot);

        double v_dot = limitV(tau, v);
        double w_dot = limitW(tau, w);


        state_dot[near_identity::X_IND] = x_dot;
        state_dot[near_identity::Y_IND] = y_dot;
        state_dot[near_identity::THETA_IND] = theta_dot;
        state_dot[near_identity::V_IND] = v_dot;
        state_dot[near_identity::W_IND] = w_dot;
        state_dot[near_identity::LAMBDA_IND] = lambda_dot;
        //std::vector<double> state_dot = {x_dot, y_dot, theta_dot, v_dot, w_dot, lambda_dot};

    }
    
    inline
    Eigen::Vector2d getTau( double x, double y, double theta, double v, double w, double lambda, double x_d, double y_d, double x_d_dot, double y_d_dot )
    {
    
        Eigen::Matrix2d R;
        R << cos(theta), -sin(theta), 
             sin(theta), cos(theta);
             
        double lambda_dot = -c_lambda_*(lambda - epsilon_);
        return getTau(x,y,theta,v,w,lambda,x_d,y_d,x_d_dot,y_d_dot,R,lambda_dot);
    }
    
    
    inline
    Eigen::Vector2d getTau( double x, double y, double theta, double v, double w, double lambda, double x_d, double y_d, double x_d_dot, double y_d_dot, const Eigen::Matrix2d R, double lambda_dot )
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
        Eigen::Vector2d u = -c_p_*(q - xy_d) - c_d_*(p - xy_d_dot);

   //     std::cout << "u: " << u(0) << ", " << u(1) << std::endl;

        //Now find tau
        Eigen::Vector2d tau = R_lambda_inv*u - w_hat*vw - lambda_dot*(w_hat - c_lambda_*Eigen::Matrix2d::Identity())*Eigen::Matrix<double, 2, 1>::Identity();
        
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
    double limitV(Eigen::Vector2d tau, double v)
    {
        return applyLimits(tau[0], v, -v_max_, v_max_, -a_max_, a_max_);
    }
    
    inline
    double limitW(Eigen::Vector2d tau, double w)
    {
        return applyLimits(tau[1], w, -w_max_, w_max_, -w_dot_max_, w_dot_max_);
    }
};


#endif  /* !near_identity.h sen */

