#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;

Simulator::Simulator() {
    // initialize the particles
    mParticles.resize(2);
    
    // Init particle positions (default is 0, 0, 0)
    mParticles[0].mPosition[0] = 0.2;
    mParticles[1].mPosition[0] = 0.2;
    mParticles[1].mPosition[1] = -0.1;
    
    mTimeStep = 0.0003;
    method = 1;
}

int Simulator::getNumParticles() {
    return mParticles.size();
}

Particle* Simulator::getParticle(int index) {
    return &mParticles[index];
}

double Simulator::getTimeStep() {
    return mTimeStep;
}

void Simulator::reset() {
    mParticles[0].mPosition[1] = 0.0;
    mParticles[0].mPosition[0] = 0.2;
    mParticles[1].mPosition[0] = 0.2;
    mParticles[1].mPosition[1] = -0.1;
    
    for (int i = 0; i < getNumParticles(); i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mAccumulatedForce.setZero();
    }
    
}

void Simulator::simulate() {
    // TODO:
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce[1] -= 9.8 * mParticles[i].mMass;
    }

	VectorXd q(6);
    VectorXd qdot(6);
    VectorXd Q(6);
    VectorXd Qhat(6);
    MatrixXd W(6,6);
    VectorXd lambda(2);
    MatrixXd J(2,6);
    MatrixXd Jdot(2,6);
    VectorXd C(2);
    VectorXd Cdot(2);

    //fill in the matrices
    q[0] = mParticles[0].mPosition[0];
    q[1] = mParticles[0].mPosition[1];
    q[2] = mParticles[0].mPosition[2];
    q[3] = mParticles[1].mPosition[0];
    q[4] = mParticles[1].mPosition[1];
    q[5] = mParticles[1].mPosition[2];

    qdot[0] = mParticles[0].mVelocity[0];
    qdot[1] = mParticles[0].mVelocity[1];
    qdot[2] = mParticles[0].mVelocity[2];
    qdot[3] = mParticles[1].mVelocity[0];
    qdot[4] = mParticles[1].mVelocity[1];
    qdot[5] = mParticles[1].mVelocity[2];

    Q[0] = 0;
    Q[1] = mParticles[0].mAccumulatedForce[1];
    Q[2] = 0;
    Q[3] = 0;
    Q[4] = mParticles[1].mAccumulatedForce[1];
    Q[5] = 0;

    W = MatrixXd::Identity(6,6)/mParticles[0].mMass;

    J(0,0) = mParticles[0].mPosition[0];
    J(0,1) = mParticles[0].mPosition[1];
    J(0,2) = mParticles[0].mPosition[2];
	J(0,3) = 0;
	J(0,4) = 0;
	J(0,5) = 0;
	J(1,0) = mParticles[0].mPosition[0] - mParticles[1].mPosition[0];
    J(1,1) = mParticles[0].mPosition[1] - mParticles[1].mPosition[1];
    J(1,2) = mParticles[0].mPosition[2] - mParticles[1].mPosition[2];
    J(1,3) = mParticles[1].mPosition[0] - mParticles[0].mPosition[0];
    J(1,4) = mParticles[1].mPosition[1] - mParticles[0].mPosition[1];
    J(1,5) = mParticles[1].mPosition[2] - mParticles[0].mPosition[2];

    Jdot(0,0) = mParticles[0].mVelocity[0];
    Jdot(0,1) = mParticles[0].mVelocity[1];
    Jdot(0,2) = mParticles[0].mVelocity[2];
	Jdot(0,3) = 0;
	Jdot(0,4) = 0;
	Jdot(0,5) = 0;
	Jdot(1,0) = mParticles[0].mVelocity[0] - mParticles[1].mVelocity[0];
    Jdot(1,1) = mParticles[0].mVelocity[1] - mParticles[1].mVelocity[1];
    Jdot(1,2) = mParticles[0].mVelocity[2] - mParticles[1].mVelocity[2];
    Jdot(1,3) = mParticles[1].mVelocity[0] - mParticles[0].mVelocity[0];
    Jdot(1,4) = mParticles[1].mVelocity[1] - mParticles[0].mVelocity[1];
    Jdot(1,5) = mParticles[1].mVelocity[2] - mParticles[0].mVelocity[2];

    C[0] = 0.5*mParticles[0].mPosition.dot(mParticles[0].mPosition) - 0.5*0.2*0.2;
    Vector3d v1 = mParticles[0].mPosition - mParticles[1].mPosition;
    C[1] = 0.5*v1.dot(v1) - 0.5*0.1*0.1;

    Cdot[0] = (mParticles[0].mPosition).dot(mParticles[0].mVelocity);
    Vector3d v = mParticles[0].mPosition - mParticles[1].mPosition;
    Cdot[1] = v.dot(mParticles[0].mVelocity - mParticles[1].mVelocity);

    MatrixXd m(2,2);
    m = J*W*J.transpose();
    lambda = m.inverse()*(-1*Jdot*qdot - J*W*Q - 50*C - 30*Cdot);
    Qhat = J.transpose()*lambda;

    mParticles[0].mAccumulatedForce[0] += Qhat[0];
    mParticles[0].mAccumulatedForce[1] += Qhat[1];
    mParticles[0].mAccumulatedForce[2] += Qhat[2];
    mParticles[1].mAccumulatedForce[0] += Qhat[3];
    mParticles[1].mAccumulatedForce[1] += Qhat[4];
    mParticles[1].mAccumulatedForce[2] += Qhat[5];

    //Euler Method
    if(method == 2)
    {
    	for (int i = 0; i < mParticles.size(); i++) {
	        mParticles[i].mPosition += mParticles[i].mVelocity * mTimeStep;
	        mParticles[i].mVelocity += mParticles[i].mAccumulatedForce / mParticles[i].mMass * mTimeStep;
	    }
	}
    
    //Midpoint Method
    if(method == 1)
	{    	
	    for (int i = 0; i < mParticles.size(); i++) {
	        mParticles[i].mPosition += mParticles[i].mVelocity * mTimeStep + 0.5 * mTimeStep * mTimeStep * mParticles[i].mAccumulatedForce / mParticles[i].mMass;
	        mParticles[i].mVelocity += mParticles[i].mAccumulatedForce / mParticles[i].mMass * mTimeStep;
	    }
	}

    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mAccumulatedForce.setZero();
    }
}