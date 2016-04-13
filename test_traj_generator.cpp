#include "traj_generator.h"

#include <iostream>
#include <sstream>
#include <vector>
#include <chrono>
#include <math.h>


//[ rhs_class
/* The rhs of x' = f(x) defined as a class */
class sample_traj_func : public virtual traj_func{

    double m_amp;
    double m_f;

public:
    sample_traj_func( double amp, double f ) : m_amp(amp), m_f(f) { }

    void dState ( const state_type &x , state_type &dxdt , const double  t  )
    {
        dxdt[6] = 1;
        dxdt[7] = sin(t*2.0*3.14*m_f) * m_amp;
    }
};
//]



int main(int  argc  , char**  argv  )
{
    using namespace std;


    //[ initial state
    state_type x0(8); 
    x0[0] = 1.0; //x
    x0[1] =-2.0; //y
    x0[2] = 0.0; //theta
    x0[3] = 0.0; //v
    x0[4] = 0.0; //w
    x0[5] = 1.0; //lambda: must be > 0!
    x0[6] = 0.0; //x_d
    x0[7] = 0.0; //y_d
    //]
    



    //double abs_err_ = 1.0e-10 , rel_err_ = 1.0e-6 , a_x_ = 1.0 , a_dxdt_ = 1.0;
    
    traj_generator trajectory_gen;
    sample_traj_func traj(0.15,.1);
    
    traj_func* trajpntr = &traj;
    
    trajectory_gen.setFunc(trajpntr);
    
    
    
    //[ Observer samples
    vector<state_type> x_vec;
    vector<double> times;
    
    size_t steps;

    traj_params* params = trajectory_gen.getDefaultParams();
    if(argc==2)
    {

      istringstream ss(argv[1]);
      double tf;
      if (!(ss >> tf))
          cerr << "Invalid number " << argv[1] << '\n';
      else
        params->tf = tf;
    }

    //How long does the integration take? Get current time
    auto t1 = std::chrono::high_resolution_clock::now();



    steps = trajectory_gen.run(x0, x_vec, times, params);


    
    //How long did it take? 
    //Stop the clock before all of the printouts
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> fp_ms = t2 - t1;
    std::cout << "Integration took " << fp_ms.count() << " ms\n";



    std::cout<< "const observer: "  << steps << " steps. final: " << '\t' << x0[0] << '\t' << x0[1]<< std::endl;
    /* output */
    std::cout << "Time" << '\t' << "Error" << '\t' << 'x' << '\t' << 'y' << '\t' << "theta" << '\t' << 'v' << '\t' << 'w' << '\t' << "lambda" << '\t' << 'x' << '\t' << 'y' << std::endl;
    
    for( size_t i=0; i<=steps; i++ )
    {
        double error_x = x_vec[i][0] - x_vec[i][6];
        double error_y = x_vec[i][1] - x_vec[i][7];
        
        double error = sqrt(error_x*error_x + error_y*error_y);
        printf("%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\t%.4f\n", times[i], error, x_vec[i][0], x_vec[i][1], x_vec[i][2], x_vec[i][3], x_vec[i][4], x_vec[i][5], x_vec[i][6], x_vec[i][7]);
        
        //cout << times[i] << '\t' << x_vec[i][0] << '\t' << x_vec[i][1] << "\t\t" << x_vec[i][2]<< "\t\t" << x_vec[i][3]<< "\t\t" << x_vec[i][4]<< '\t' << x_vec[i][5]<< '\t' << x_vec[i][6]<< '\t' << x_vec[i][7] << '\n';
    }
    //]




    


}
