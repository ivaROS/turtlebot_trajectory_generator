/*
 Copyright 2010-2012 Karsten Ahnert
 Copyright 2011-2013 Mario Mulansky
 Copyright 2013 Pascal Germroth
 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#include <iostream>
#include <vector>

#include <boost/numeric/odeint.hpp>
#include "near_identity.hpp"



//[ rhs_function
/* The type of container used to hold the state vector */
typedef std::vector< double > state_type;


//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class traj_gen {

    double m_amp;
    double m_f;

public:
    traj_gen( double amp, double f ) : m_amp(amp), m_f(f) { }

    void operator() ( const state_type &x , state_type &dxdt , const double  t  )
    {
        dxdt[6] = 1;
        dxdt[7] = std::sin(t*2.0*3.14*m_f) * m_amp / (2*3.14*m_f);
    }
};
//]




//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class ni_controller {

    near_identity ni_;
    traj_gen traj_;
    

public:
    ni_controller( near_identity ni, traj_gen traj) :  ni_(ni), traj_(traj) { }

    void operator() ( const state_type &x , state_type &dxdt , const double  t  )
    {
        traj_(x,dxdt,t);
        ni_(x,dxdt,t);
    }
};






//[ integrate_observer
struct push_back_state_and_time
{
    std::vector< state_type >& m_states;
    std::vector< double >& m_times;

    push_back_state_and_time( std::vector< state_type > &states , std::vector< double > &times )
    : m_states( states ) , m_times( times ) { }

    void operator()( const state_type &x , double t )
    {
        m_states.push_back( x );
        m_times.push_back( t );
    }
};
//]



int main(int /* argc */ , char** /* argv */ )
{
    using namespace std;
    using namespace boost::numeric::odeint;


    //[ state_initialization
    state_type x0(8), dx(8);  //x,y,theta,v,w,lambda, xd,yd
    x0[0] = 0.0; //x0
    x0[1] = 0.0; //y0
    x0[2] = 0.0; //theta0
    x0[3] = 0.0; //v0
    x0[4] = 0.0; //w0
    x0[5] = 0.5; //lambda0
    x0[6] = 0.0; //xd0
    x0[7] = 0.0; //yd0
    //]
    
    dx[0] = 0.0; //x0
    dx[1] = 0.0; //y0
    dx[2] = 0.0; //theta0
    dx[3] = 0.0; //v0
    dx[4] = 0.0; //w0
    dx[5] = 0.5; //lambda0
    dx[6] = 0.0; //xd0
    dx[7] = 0.0; //yd0
    
    
    near_identity ni(1,2,1,.01);

    
    ni(x0,dx,0);
    
    std::cout << "[" << 
    dx[0] << ", " << 
    dx[1] << ", " << 
    dx[2] << ", " <<
    dx[3] << ", " <<
    dx[4] << ", " <<
    dx[5] << "]" << std::endl;

///

    dx[6] = 1;
    
    
    ni(x0,dx,0);
    
    std::cout << "[" << 
    dx[0] << ", " << 
    dx[1] << ", " << 
    dx[2] << ", " <<
    dx[3] << ", " <<
    dx[4] << ", " <<
    dx[5] << "]" << std::endl;

///

    dx[6] = 0;
    dx[7] = 1;
    
    
    ni(x0,dx,0);
    
    std::cout << "[" << 
    dx[0] << ", " << 
    dx[1] << ", " << 
    dx[2] << ", " <<
    dx[3] << ", " <<
    dx[4] << ", " <<
    dx[5] << "]" << std::endl;

///

    x0[0] = 1.0; 
    x0[1] = 2.0; 
    x0[2] = 1.0; 

    ni(x0,dx,0);
    
    std::cout << "[" << 
    dx[0] << ", " << 
    dx[1] << ", " << 
    dx[2] << ", " <<
    dx[3] << ", " <<
    dx[4] << ", " <<
    dx[5] << "]" << std::endl;
    
///


    x0[3] = 1.0; 
    x0[4] = 1.0; 


    ni(x0,dx,0);
    
    std::cout << "[" << 
    dx[0] << ", " << 
    dx[1] << ", " << 
    dx[2] << ", " <<
    dx[3] << ", " <<
    dx[4] << ", " <<
    dx[5] << "]" << std::endl;
    
    
///   

    x0[6] = 5.0; 
    x0[7] = 1.0; 


    ni(x0,dx,0);
    
    std::cout << "[" << 
    dx[0] << ", " << 
    dx[1] << ", " << 
    dx[2] << ", " <<
    dx[3] << ", " <<
    dx[4] << ", " <<
    dx[5] << "]" << std::endl;
}
