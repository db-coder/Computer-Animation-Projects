/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Karen Liu <karenliu@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.h"

Controller::Controller(dart::dynamics::SkeletonPtr _skel, dart::constraint::ConstraintSolver* _constrSolver, double _t) {
  mSkel = _skel;
  mConstraintSolver = _constrSolver;
  mTimestep = _t;

  int nDof = mSkel->getNumDofs();
  mKp = Eigen::MatrixXd::Identity(nDof, nDof);
  mKd = Eigen::MatrixXd::Identity(nDof, nDof);
  mTorques.resize(nDof);
  mDefaultPose.resize(nDof);
  mDesiredDofs.resize(nDof);
  
  // Set default pose as the initial pose when the controller is instantiated
  mDefaultPose = mSkel->getPositions();
  mDesiredDofs = mDefaultPose;
  
  mTorques.setZero();

  // Using SPD results in simple spring coefficients
  for (int i = 0; i < nDof; i++)
    mKp(i, i) = 400.0;
  for (int i = 0; i < nDof; i++)
    mKd(i, i) = 40.0;

  // Global dofs don't have PD control
  for (int i = 0; i < 6; i++) {
    mKp(i, i) = 0.0;
    mKd(i, i) = 0.0;
  }

  // Make shoulders and elbows loose
  std::vector<int> dofIndex;
  dofIndex.push_back((mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_left_x")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_forearm_left")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_bicep_right_x")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_forearm_right")->getIndexInSkeleton()));
  for (int i = 0; i < dofIndex.size(); i++) {
    int index = dofIndex[i];
    mKp(index, index) = 20.0;
    mKd(index, index) = 2.0;
  }

  // Make wrists even looser
  dofIndex.clear();
  dofIndex.push_back((mSkel->getDof("j_hand_left_1")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_hand_left_2")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_hand_right_1")->getIndexInSkeleton()));
  dofIndex.push_back((mSkel->getDof("j_hand_right_2")->getIndexInSkeleton()));
  for (int i = 0; i < dofIndex.size(); i++) {
    int index = dofIndex[i];
    mKp(index, index) = 1.0;
    mKd(index, index) = 0.1;
  }

  for (int i = 0; i < nDof; i++)
    mSkel->getDof(i)->setDampingCoefficient(0.01);
  mPreOffset = 0.0;
  mLeftHandHold = NULL;
  mRightHandHold = NULL;
  mFootContact = NULL;
  mLeftHandContact = NULL;
  mRightHandContact = NULL;
  mTimer = 300;
  mState = "STAND";

  Max = 0;
  Min = INT_MAX;
  MaxTime = 0;
  MinTime = 0;
  max_found = false;
  min_found = false;
  counter = 0;
  movingAway = false;
  temp = 0;
  PrevLinVel = 0;
  MaxTimeFinal = 0;
  MinTimeFinal = 0;
  goAhead = false;
}

Controller::~Controller() {
}

Eigen::VectorXd Controller::getTorques() {
  return mTorques;
}

double Controller::getTorque(int _index) {
  return mTorques[_index];
}

void Controller::setDesiredDof(int _index, double _val) {
  mDesiredDofs[_index] = _val;
}

void Controller::computeTorques(int _currentFrame) {
  mCurrentFrame = _currentFrame;
  mTorques.setZero();
  if (mState == "STAND") {
    stand();
  } else if (mState == "CROUCH") {
    crouch();
  } else if (mState == "JUMP") {
    jump();
  } else if (mState == "REACH") {
    reach();
  } else if (mState == "GRAB") {
    grab();
  } else if (mState == "RELEASE") {
    release();
  } else if (mState == "SWING") {
    swing();
  } else if (mState == "RESET") {
    reset();
  } else {
    std::cout << "Illegal state: " << mState << std::endl;
  }

  // Just to make sure no illegal torque is used. Do not remove this.
  for (int i = 0; i < 6; i++) {
    mTorques[i] = 0.0;
  }
}

void Controller::checkContactState() {
  mFootContact = NULL;
  mLeftHandContact = NULL;
  mRightHandContact = NULL;
  dart::collision::CollisionDetector* cd = mConstraintSolver->getCollisionDetector();
  int nContacts = cd->getNumContacts();
  for (int i = 0; i < nContacts; i++) {
    dart::dynamics::BodyNodePtr body1 = cd->getContact(i).bodyNode1.lock().get();
    dart::dynamics::BodyNodePtr body2 = cd->getContact(i).bodyNode2.lock().get();
  
    if (body1 == mSkel->getBodyNode("h_heel_left") || body1 == mSkel->getBodyNode("h_heel_left")
        || body1 == mSkel->getBodyNode("h_heel_right") || body1 == mSkel->getBodyNode("h_heel_right"))
      mFootContact = body2;
    if (body2 == mSkel->getBodyNode("h_heel_left") || body2 == mSkel->getBodyNode("h_heel_left")
        || body2 == mSkel->getBodyNode("h_heel_right") || body2 == mSkel->getBodyNode("h_heel_right"))
      mFootContact = body1;
    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_left"))
      mLeftHandContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_left"))
      mLeftHandContact = body1;
    if (body1->isCollidable() && body1 == mSkel->getBodyNode("h_hand_right"))
      mRightHandContact = body2;
    if (body2->isCollidable() && body2 == mSkel->getBodyNode("h_hand_right"))
      mRightHandContact = body1;
  }
}

void Controller::reset() {
  mSkel->resetPositions();
  mSkel->setPositions(mDefaultPose);
  mSkel->resetVelocities();
  mSkel->resetAccelerations();
  this->leftHandRelease();
  this->rightHandRelease();
  mTorques.setZero();

  int nDof = mSkel->getNumDofs();
  for (int i = 0; i < nDof; i++)
    mSkel->getDof(i)->setDampingCoefficient(0.01);
  mPreOffset = 0.0;
  mLeftHandHold = NULL;
  mRightHandHold = NULL;
  mFootContact = NULL;
  mLeftHandContact = NULL;
  mRightHandContact = NULL;
  mTimer = 300;
  mState = "STAND";

  Max = 0;
  Min = INT_MAX;
  MaxTime = 0;
  MinTime = 0;
  max_found = false;
  min_found = false;
  counter = 0;
  movingAway = false;
  temp = 0;
  MaxTimeFinal = 0;
  MinTimeFinal = 0;
  goAhead = false;
  PrevLinVel = 0;
}

void Controller::stand() {
  // Change to default standing pose
  mDesiredDofs = mDefaultPose;
  stablePD();
  ankleStrategy();
  mTimer--;

  // Switch to crouch if time is up
  if (mTimer == 0) {
    mState = "CROUCH";
    mTimer = 500;
    std::cout << mCurrentFrame << ": " << "STAND -> CROUCH" << std::endl;
  }
}

void Controller::crouch() {

  // Change to crouching pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -1.1;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -1.1;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = 0.6;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = 0.6;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;

  // After a while, lean forward
  if (mTimer < 200) {
    mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = 1.0;
    mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = 1.0;
  }

  stablePD();
  ankleStrategy();
  mTimer--;

  if (mTimer == 0) {
    mState = "JUMP";
    std::cout << mCurrentFrame << ": " << "CROUCH -> JUMP" << std::endl;

  }
}

void Controller::jump() {
  // Change to leaping pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 0.3;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -1.0;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 0.3;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 1.0;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.5;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.5;
  
  stablePD();

  // Use Jacobian transpose to compute pushing torques
  Eigen::Vector3d vf(-1100.0, -2600, 0.0);
  Eigen::Vector3d offset(0.05, -0.02, 0.0);
  virtualForce(vf, mSkel->getBodyNode("h_heel_left"), offset);
  virtualForce(vf, mSkel->getBodyNode("h_heel_right"), offset);

  checkContactState();
  if (mFootContact == NULL) {
    mState = "REACH";
    std::cout << mCurrentFrame << ": " << "JUMP -> REACH" << std::endl;
  }
}

void Controller::reach() {
  // Change to reaching pose
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.3;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.3;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.4;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.4;
  stablePD();

  checkContactState();
  if (mFootContact) { // If feet are in contact again, go back to JUMP and continue to push
    mState = "JUMP";
    std::cout << mCurrentFrame << ": " << "REACH -> JUMP" << std::endl;
  } else if (mLeftHandContact || mRightHandContact) {
    mState = "GRAB";
    mTimer = 500;
    std::cout << mCurrentFrame << ": " << "REACH -> GRAB" << std::endl;
  } else {
    mState = "REACH";
  }
}

void Controller::grab() {
  leftHandGrab();
  rightHandGrab();

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.2;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -0.2;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = -2.3;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 0.7;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 2.3;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0.4;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0.4;
  stablePD();
  mTimer--;

  if (mTimer == 0) {
    mState = "SWING";
    std::cout << mCurrentFrame << ": " << "GRAB -> SWING" << std::endl;
  }
}  

void Controller::swing() {
  // TODO: Need a better controller to increase the speed
  // and land at the right moment
  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_bicep_left_z")->getIndexInSkeleton()] = 0;
  mDesiredDofs[mSkel->getDof("j_bicep_left_y")->getIndexInSkeleton()] = 0;
  mDesiredDofs[mSkel->getDof("j_bicep_left_x")->getIndexInSkeleton()] = 0;
  mDesiredDofs[mSkel->getDof("j_bicep_right_z")->getIndexInSkeleton()] = 0;
  mDesiredDofs[mSkel->getDof("j_bicep_right_y")->getIndexInSkeleton()] = 0;
  mDesiredDofs[mSkel->getDof("j_bicep_right_x")->getIndexInSkeleton()] = 0;
  mDesiredDofs[mSkel->getDof("j_scapula_left")->getIndexInSkeleton()] = 0;
  mDesiredDofs[mSkel->getDof("j_scapula_right")->getIndexInSkeleton()] = 0;
  mDesiredDofs[mSkel->getDof("j_forearm_left")->getIndexInSkeleton()] = 0;
  mDesiredDofs[mSkel->getDof("j_forearm_right")->getIndexInSkeleton()] = 0;
    
  Eigen::Vector3d AngVel = mSkel->getBodyNode("h_forearm_left")->getLinearVelocity();

  if(AngVel[0] > 0) {
    mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = 0.75;
    mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = 0.75;
    mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = 0.75;
    mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = 0.75;
    mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = 0;
    mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = 0;
    mDesiredDofs[mSkel->getDof("j_hand_left_1")->getIndexInSkeleton()] = 0.75;
    mDesiredDofs[mSkel->getDof("j_hand_right_1")->getIndexInSkeleton()] = 0.75;
  }
  else {
    mDesiredDofs[mSkel->getDof("j_thigh_left_z")->getIndexInSkeleton()] = -0.75;
    mDesiredDofs[mSkel->getDof("j_thigh_right_z")->getIndexInSkeleton()] = -0.75;
    mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -0.75;
    mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -0.75;
    mDesiredDofs[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] = 0;
    mDesiredDofs[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] = 0;
    mDesiredDofs[mSkel->getDof("j_hand_left_1")->getIndexInSkeleton()] = -0.75;
    mDesiredDofs[mSkel->getDof("j_hand_right_1")->getIndexInSkeleton()] = -0.75;
  }

  stablePD();

  // TODO: Figure out the condition to release the bar
  int mWinWidth = 640;
  int temp_max = 0;

  if(counter < 2) {
    for(int i = InputSensor.size()-4; i >=0; i -= 4) {
      if((int)InputSensor[i] == 92) {
        temp_max = floor(i/(4*mWinWidth));
      }
    }
    if(counter == 1) {
      if(temp < temp_max){  //moving away
        movingAway = true;
      }
      else {
        movingAway = false;
      }
    }

    temp = temp_max;
    counter++;
  }
  else {
    if(!(max_found && min_found)) {
      if(movingAway) {
        if(!max_found && mCurrentFrame%10 == 0) {
          int temp_max = 0, i;
          for(i = InputSensor.size()-4; i >=0; i -= 4) {
            if((int)InputSensor[i] == 92 && (int)InputSensor[i+1] == 82 && (int)InputSensor[i+2] == 92) {
              temp_max = floor(i/(4*mWinWidth));
              break;
            }
          }
          Max = std::max(Max,temp_max);
        }

        if(!max_found && temp_max != 0 && temp_max < Max) {
          MaxTime = mCurrentFrame;
          max_found = true;
        }

        int temp_min = 0, i;
        if(max_found && !min_found && mCurrentFrame%10 == 0) {
          for(i = 0; i < InputSensor.size()-4; i += 4) {
            if((int)InputSensor[i] == 92 && (int)InputSensor[i+1] == 82 && (int)InputSensor[i+2] == 92) {
              temp_min = floor(i/(4*mWinWidth));
              break;
            }
          }
          if(temp_min != 0) {
            Min = std::min(Min,temp_min);
          }
        }

        if(max_found && !min_found && temp_min != 0 && temp_min > Min) {
          MinTime = mCurrentFrame;
          min_found = true;
        }
      }
      else {
        int temp_min = 0, i;
        if(!min_found && mCurrentFrame%10 == 0) {
          for(i = 0; i < InputSensor.size()-4; i += 4) {
            if((int)InputSensor[i] == 92 && (int)InputSensor[i+1] == 82 && (int)InputSensor[i+2] == 92) {
              temp_min = floor(i/(4*mWinWidth));
              break;
            }
          }
          if(temp_min != 0) {
            Min = std::min(Min,temp_min);
          }
        }

        if(!min_found && temp_min != 0 && temp_min > Min) {
          MinTime = mCurrentFrame;
          min_found = true;
        }

        int temp_max = 0;
        if(min_found && !max_found && mCurrentFrame%10 == 0) {
          for(i = InputSensor.size()-4; i >=0; i -= 4) {
            if((int)InputSensor[i] == 92 && (int)InputSensor[i+1] == 82 && (int)InputSensor[i+2] == 92) {
              temp_max = floor(i/(4*mWinWidth));
              break;
            }
          }
          Max = std::max(Max,temp_max);
        }

        if(min_found && !max_found && temp_max != 0 && temp_max < Max) {
          MaxTime = mCurrentFrame;
          max_found = true;
        }
      }
    }
  }

  if(max_found && min_found) {

    MinTimeFinal = MinTime;
    MaxTimeFinal = MaxTime;
    goAhead = true;

    // Max = 0;
    // Min = INT_MAX;
    // MaxTime = 0;
    // MinTime = 0;
    // // max_found = false;
    // // min_found = false;
    // counter = 0;
    // movingAway = false;
    // temp = 0;
  }

  int cycleTime = std::abs(2*(MaxTimeFinal - MinTimeFinal));
  // std::cout<<cycleTime<<std::endl;
  int timeToFall = (9540-cycleTime*3)/76;
  int range = 0;

  if(goAhead) {
    bool jump = false;
    Eigen::Vector3d LinVel = mSkel->getBodyNode("h_forearm_left")->getLinearVelocity();
    if(LinVel[0] > 0.2) {
      if(PrevLinVel > LinVel[0]) {

        int x = 1;

        while(true) {
          int mCurrentFrameMax = MinTimeFinal - timeToFall + x*cycleTime;
          int mCurrentFrameMin = mCurrentFrameMax - range;

          if(mCurrentFrame >= mCurrentFrameMin && mCurrentFrame <= mCurrentFrameMax) {
            jump = true;
            break;
          }
          else if(mCurrentFrame < mCurrentFrameMin)
            break;
          // if(MinTime - timeToFall + x*cycleTime == mCurrentFrame){
          //   jump = true;
          //   break;
          // }
          // else if(MinTime - timeToFall + x*cycleTime > mCurrentFrame)
          //   break;
          x++;
        }
      }
    }
    PrevLinVel = LinVel[0];

    if (jump) {
      mState = "RELEASE";
      std::cout << mCurrentFrame << ": " << "SWING -> RELEASE" << std::endl;
    }
  }
}

void Controller::release() {
  leftHandRelease();
  rightHandRelease();

  mDesiredDofs = mDefaultPose;
  mDesiredDofs[mSkel->getDof("j_abdomen_2")->getIndexInSkeleton()] = -1.5;
  mDesiredDofs[mSkel->getDof("j_shin_left")->getIndexInSkeleton()] = -1.5;
  mDesiredDofs[mSkel->getDof("j_shin_right")->getIndexInSkeleton()] = -1.5;
  stablePD();
}
  
void Controller::stablePD() {
  Eigen::VectorXd q = mSkel->getPositions();
  Eigen::VectorXd dq = mSkel->getVelocities();
  Eigen::VectorXd constrForces = mSkel->getConstraintForces();
  Eigen::MatrixXd invM = (mSkel->getMassMatrix() + mKd * mTimestep).inverse();
  Eigen::VectorXd p = -mKp * (q + dq * mTimestep - mDesiredDofs);
  Eigen::VectorXd d = -mKd * dq;
  Eigen::VectorXd qddot =
      invM * (-mSkel->getCoriolisAndGravityForces() + p + d + constrForces);

  mTorques += p + d - mKd * qddot * mTimestep;
}

void Controller::ankleStrategy() {
  Eigen::Vector3d com = mSkel->getCOM();
  Eigen::Vector3d cop = mSkel->getBodyNode("h_heel_left")->getTransform()
                        * Eigen::Vector3d(0.05, 0, 0);
  double offset = com[0] - cop[0];
   if (offset < 0.1 && offset > 0.0) {
    double k1 = 200.0;
    double k2 = 100.0;
    double kd = 10.0;
    mTorques[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_left")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_right")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  } else if (offset > -0.2 && offset < -0.05) {
    double k1 = 2000.0;
    double k2 = 100.0;
    double kd = 100.0;
    mTorques[mSkel->getDof("j_heel_left_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_left")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_heel_right_1")->getIndexInSkeleton()] += -k1 * offset + kd * (mPreOffset - offset);
    mTorques[mSkel->getDof("j_toe_right")->getIndexInSkeleton()] += -k2 * offset + kd * (mPreOffset - offset);
    mPreOffset = offset;
  }  
}

void Controller::virtualForce(Eigen::Vector3d _force, dart::dynamics::BodyNode* _bodyNode, Eigen::Vector3d _offset) {
  Eigen::MatrixXd jacobian = mSkel->getLinearJacobian(_bodyNode, _offset);
  mTorques += jacobian.transpose() * _force;
}

void Controller::leftHandGrab() {  
  if (mLeftHandHold != NULL)
    return;
  checkContactState();
  if (mLeftHandContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_hand_left");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mLeftHandContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mLeftHandHold = hold;
}

void Controller::leftHandRelease() {
  if (mLeftHandHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mLeftHandHold);
  mSkel->getBodyNode("h_hand_left")->setCollidable(true);
  mLeftHandHold = NULL;
}

void Controller::rightHandGrab() {  
  if (mRightHandHold != NULL)
    return;

  checkContactState();
  if (mRightHandContact == NULL)
    return;
  dart::dynamics::BodyNode* bd = mSkel->getBodyNode("h_hand_right");
  dart::constraint::WeldJointConstraint *hold = new dart::constraint::WeldJointConstraint(bd, mRightHandContact);
  mConstraintSolver->addConstraint(hold);
  bd->setCollidable(false);
  mRightHandHold = hold;
}

void Controller::rightHandRelease() {
  if (mRightHandHold == NULL)
    return;
  mConstraintSolver->removeConstraint(mRightHandHold);
  mSkel->getBodyNode("h_hand_right")->setCollidable(true);
  mRightHandHold = NULL;
}

void Controller::setState(std::string _state) {
  mState = _state;
}

dart::dynamics::SkeletonPtr Controller::getSkel() {
  return mSkel;
}

Eigen::VectorXd Controller::getDesiredDofs() {
  return mDesiredDofs;
}

Eigen::MatrixXd Controller::getKp() {
  return mKp;
}

Eigen::MatrixXd Controller::getKd() {
  return mKd;
}

