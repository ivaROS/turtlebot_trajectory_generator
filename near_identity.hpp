#include <Eigen/Dense>

typedef std::vector< double > state_type;

class near_identity {

    double c_p_;
    double c_d_;
    double c_lambda_;
    double epsilon_;

public:
    near_identity( double c_p, double c_d, double c_lambda, double epsilon ) : c_p_(c_p), c_d_(c_d), c_lambda_(c_lambda), epsilon_(epsilon) { }

    void operator() ( const state_type &state , state_type &state_dot, const double /* t*/  )
    {
        //load state variables
        double x = state(0)
        double y = state(1);
        double theta = state(2);
        double v = state(3);
        double w = state(4);
        double lambda = state(5);


        /*Compute desired characteristics of path for this time
        x_d = f_t(t);
        x_d_dot = f_dot_t(t);
        */

        double x_d = state(6);
        double y_d = state(7);
        double x_d_dot = state_dot(6);
        double y_d_dot = state_dot(7);

        Eigen::Vector2d xy_d << x_d, y_d;
        Eigen::Vector2d xy_d_dot << x_d_dot, y_d_dot;

        //Compute rotation matrices R, R_lambda, R_lambda_inv
        Eigen::Matrix2d R <<  cos(theta), -sin(theta), 
                              sin(theta), cos(theta);

        //R_lambda = [R*e1 lambda*R*e2];
        Eigen::Matrix2d R_lambda << R.at(0,0), lambda*R.at(0,1), R.at(1,0), lambda*R.at(1,1);

        //R_lambda_inv = [R*e1 R*e2/lambda]';
        Eigen::Matrix2d R_lambda_inv << R.at(0,0), R.at(1,0), R.at(0,1)/lambda, R.at(1,1)/lambda;


        double lambda_dot = -c_lambda_*(lambda - epsilon_);

        Eigen::Matrix2d w_hat << 0, -lambda*w, (1.0/lambda)*w, lambda_dot/lambda;

        //q = xy + lambda*R*e1;
        Eigen::Vector2d q << x + lambda*R.at(0,0), y + lambda*R.at(1,0);

        //p = R_lambda*v + lambda_dot*R_lambda*e1;
        Eigen::Vector2d p << v + lambda_dot*R_lambda.at(0,0), w + lambda_dot*R_lambda.at(1,0);

        
        
        
        
        //u can be any expression that will feedback stabilize a linear system
        //here, using 2nd order feedback controller   ; + x_d_dot_dot
        Eigen::Vector2d u = -c_p_*(q - x_d) - c_d_*(p - x_d_dot);


        //Now find tau
        Eigen::Vector2d tau = R_lambda_inv*u - w_hat*v - lambda_dot*(w_hat - c_lambda*eye(n))*e1;


        //Now find derivatives of state variables
        //x_dot = (R*e1)*v1;
        double x_dot = R.at(0,0)*v;
        double y_dot = R.at(1,0)*v;
        double v_dot = tau.at(0);
        double w_dot = tau.at(1);
        double theta_dot = w;

        state_dot[0] = x_dot;
        state_dot[1] = y_dot;
        state_dot[2] = theta_dot;
        state_dot[3] = v_dot;
        state_dot[4] = w_dot;
        state_dot[5] = lambda_dot;
        //std::vector<double> state_dot = {x_dot, y_dot, theta_dot, v_dot, w_dot, lambda_dot};

    }
};



