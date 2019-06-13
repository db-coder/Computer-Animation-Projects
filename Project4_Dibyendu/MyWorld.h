#ifndef _MYWORLD_
#define _MYWORLD_

#include <vector>

#define IX(i, j) ((i)+(getNumCells()+2)*(j))
#define SWAPPING(x0,x) {double *tmp=x0;x0=x;x=tmp;}

class MyWorld {
 public:
    MyWorld();

    virtual ~MyWorld();

    void initialize(int _numCells, double _timeStep, double _diffCoef, double _viscCoef);
    double getTimeStep();
    int getNumCells() { return mNumCells;}
    double getDensity(int _c, int _index) { if(_c == 0) return mDensityR[_index]; else if(_c == 1) return mDensityG[_index]; else return mDensityB[_index]; }
    double getVelocityU(int _index) { return mU[_index]; }
    double getVelocityV(int _index) { return mV[_index]; }
    void setDensity(int _i, int _j, int _c, double _source) { if(_c == 0) mDensityR[IX(_i, _j)] += mTimeStep * _source; else if(_c == 1) mDensityG[IX(_i, _j)] += mTimeStep * _source; else mDensityB[IX(_i, _j)] += mTimeStep * _source; }
    void setU(int _i, int _j, double _force) { mU[IX(_i, _j)] += mTimeStep * _force; }
    void setV(int _i, int _j, double _force) { mV[IX(_i, _j)] += mTimeStep * _force; }
    
    void simulate();
    
 protected:
    void densityStep(double *_x, double *_x0);
    void velocityStep(double *_u, double *_v, double *_u0, double *_v0);
    void diffuseDensity(double *_x, double *_x0);
    void diffuseVelocity(double *_u, double *_v, double *_u0, double *_v0);
    void advectDensity(double *_d, double *_d0, double *_u, double *_v);
    void advectVelocity(double *_u, double *_v, double *_u0, double *_v0);
    void project(double *_u, double *_v, double *_u0, double *_v0);
    void externalForces();
    void linearSolve(double *_x, double *_x0, double _a, double _c);
    void setBoundary(double *_x);
    void setVelocityBoundary(double *_u, double *_v);

    int mNumCells;
    double mTimeStep;
    double mDiffusionCoef;
    double mViscosityCoef;
    double *mU;
    double *mV;
    double *mPreU;
    double *mPreV;
    double *mDensityR;
    double *mDensityG;
    double *mDensityB;
    double *mPreDensityR;
    double *mPreDensityG;
    double *mPreDensityB;
};

#endif
