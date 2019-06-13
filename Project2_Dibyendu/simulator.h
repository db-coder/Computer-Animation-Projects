#ifndef SIMULATOR_H
#define SIMULATOR_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <Eigen/Dense>
#include <vector>
#include <string>

#include "particle.h"

using namespace Eigen;

// int m = 2, n = 2;
// class containing objects to be simulated
class Simulator {
public:
	// VectorXd q(6);
 //    VectorXd qdot(6);
 //    VectorXd Q(6);
 //    VectorXd Qhat(6);
 //    MatrixXd W(6,6);
 //    VectorXd lambda(2);
 //    MatrixXd J(2,6);
 //    MatrixXd Jdot(2,6);
 //    VectorXd C(2);
 //    VectorXd Cdot(2);
    
    Simulator();
        
    void simulate();
    
    int getNumParticles();
    
    Particle* getParticle(int);
    
    double getTimeStep();
    
    void reset();
    int method;

private:
    double mTimeStep;       // time step
    std::vector<Particle> mParticles;
};

#endif  // SIMULATOR_H
