#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;

Simulator::Simulator() {
    // initialize the particles
    mParticles.resize(4);
    
    // Init particle positions (default is 0, 0, 0)
    mParticles[0].mPosition[0] = -0.3;
    mParticles[0].mPosition[1] = 20.0;
    mParticles[1].mPosition[0] = 0.0;
    mParticles[1].mPosition[1] = 20.0;
    mParticles[2].mPosition[0] = 0.3;
    mParticles[2].mPosition[1] = 20.0;
    mParticles[3].mPosition[0] = 0.6;
    mParticles[3].mPosition[1] = 20.0;
    
    // Init particle colors (default is red)
    mParticles[1].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
    mParticles[2].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
    mParticles[3].mColor = Eigen::Vector4d(0.2, 0.9, 0.2, 1.0); // Green
    
    mTimeStep = 0.03;
    mElapsedTime = 0;
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
    mParticles[0].mPosition[0] = -0.3;
    mParticles[0].mPosition[1] = 20.0;
    mParticles[1].mPosition[0] = 0.0;
    mParticles[1].mPosition[1] = 20.0;
    mParticles[2].mPosition[0] = 0.3;
    mParticles[2].mPosition[1] = 20.0;
    mParticles[3].mPosition[0] = 0.6;
    mParticles[3].mPosition[1] = 20.0;
    
    for (int i = 0; i < 4; i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mAccumulatedForce.setZero();
    }
    
    mElapsedTime = 0;
}

void Simulator::setVelocity(float velocity) {
    reset();
    mParticles[0].mVelocity[1] = velocity;
    mParticles[1].mVelocity[1] = velocity;
    mParticles[2].mVelocity[1] = velocity;
    mParticles[3].mVelocity[1] = velocity;
}

void Simulator::simulate() {
    // TODO: Replace the following code
    
    // Analytical Solution
    // mParticles[0].mPosition[1] = 20-0.5*9.8*mElapsedTime*mElapsedTime;

    // Explicit Euler
    mParticles[1].mPosition[1] += mTimeStep*mParticles[1].mVelocity[1];
    mParticles[1].mVelocity[1] += -mTimeStep*9.8;
    
    // Midpoint Method
    mParticles[2].mPosition[1] += mTimeStep*mParticles[2].mVelocity[1] - 0.5*mTimeStep*mTimeStep*9.8;
    mParticles[2].mVelocity[1] += -mTimeStep*9.8;

    //Semi Implicit Method
    mParticles[3].mVelocity[1] += -mTimeStep*9.8;
    mParticles[3].mPosition[1] += mTimeStep*mParticles[3].mVelocity[1] - mTimeStep*mTimeStep*9.8;    
    mElapsedTime += mTimeStep;

    mParticles[0].mPosition[1] = 20 + mParticles[0].mVelocity[1]*mElapsedTime - 0.5*9.8*mElapsedTime*mElapsedTime;

    // mParticles[0].mPosition[1] = mParticles[2].mPosition[1];
}