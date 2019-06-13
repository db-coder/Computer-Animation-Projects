#include "MyWorld.h"
#include <iostream>

using namespace Eigen;
using namespace dart::dynamics;

MyWorld::MyWorld() {
  // Load a skeleton from file
  mSkel = dart::utils::SkelParser::readSkeleton(DART_DATA_PATH"skel/human.skel");

  // Create markers
  createMarkers();
  
  mC.resize(mMarkers.size());
  mJ.resize(mMarkers.size());
  mTarget.resize(mMarkers.size());
  mConstrainedMarker.resize(mMarkers.size());

  // Initialize Jacobian
  for (int i = 0; i < mMarkers.size(); ++i)
  {
    mJ[i] = MatrixXd::Zero(3, mSkel->getNumDofs());
    mConstrainedMarker[i] = -1;
  }
}

MyWorld::~MyWorld() {
}

void MyWorld::solve() {
  for (int i = 0; i < mMarkers.size(); ++i)
  {
    if (mConstrainedMarker[i] == -1)
      continue; 
    int numIter = 300;
    double alpha = 0.01;
    int nDof = mSkel->getNumDofs();
    VectorXd gradients(nDof);
    VectorXd newPose(nDof);
    for (int j = 0; j < numIter; j++) {
      gradients = updateGradients();
      newPose = mSkel->getPositions() - alpha * gradients;
      mSkel->setPositions(newPose); 
      mSkel->computeForwardKinematics(true, false, false); // DART updates all the transformations based on newPose
    } 
  }  
}

VectorXd MyWorld::updateGradients() {
  VectorXd gradients = VectorXd::Zero(mSkel->getNumDofs());

  for (int i = 0; i < mMarkers.size(); ++i)
  {
    if(mConstrainedMarker[i] == -1)
      continue;
    // compute c(q)
    mC[i] = getMarker(mConstrainedMarker[i])->getWorldPosition() - mTarget[i];

    // compute J(q)
    Vector4d offset;
    offset << getMarker(mConstrainedMarker[i])->getLocalPosition(), 1; // Create a vector in homogeneous coordinates

    BodyNode *node = getMarker(mConstrainedMarker[i])->getBodyNode();
     while (node->getParentBodyNode()) 
     {
        // Joint and Dofs
        Joint *joint = node->getParentJoint();
        int nDof = joint->getNumDofs();

        // Constant matrix in equation
        Matrix4d worldToParent = node->getParentBodyNode()->getTransform().matrix();
        Matrix4d parentToJoint = joint->getTransformFromParentBodyNode().matrix();
        Matrix4d jointToChild = joint->getTransformFromChildBodyNode().inverse().matrix();

        // compute dR and R
        Matrix4d _offset = parentToJoint;

        for (int j = 0; j < nDof; j++) 
        {
            Matrix4d _jCol = worldToParent * parentToJoint;

            if (j > 0) 
            {
                for (int k = 0; k < j; k++) 
                {
                    _jCol *= joint->getTransform(k).matrix();
                }
            }

            Matrix4d dR = joint->getTransformDerivative(j);
            _jCol *= dR;

            if (j < (nDof - 1)) 
            {
                for (int k = j + 1; k < nDof; k++) 
                {
                    _jCol *= joint->getTransform(k).matrix();
                }
            }

            Vector4d jCol = _jCol * jointToChild * offset;
            int colIndex = joint->getIndexInSkeleton(j);
            mJ[i].col(colIndex) = jCol.head(3);
            _offset *= joint->getTransform(j).matrix();
        }

        // update offset
        offset = _offset * jointToChild * offset;

        // update current node to parentNode
        node = node->getParentBodyNode();
    }
    gradients += 2 * mJ[i].transpose() * mC[i];
  }
  return gradients;
}

void MyWorld::createConstraint(int _index) {
  if (_index >= 0 && _index < mMarkers.size()) {
    mTarget[_index] = getMarker(_index)->getWorldPosition();
    mConstrainedMarker[_index] = _index;
  } 
  // else {
  //   mConstrainedMarker = -1;
  // }
}

void MyWorld::modifyConstraint(int _index, Vector3d _deltaP) {
  if (_index >= 0 && _index < mMarkers.size() && mConstrainedMarker[_index] == _index)
    mTarget[_index] += _deltaP;
}

void MyWorld::removeConstraint(int _index) {
  mConstrainedMarker[_index] = -1;
}

Marker* MyWorld::getMarker(int _index) {
  return mMarkers[_index];
}

void MyWorld::createMarkers() {
  Vector3d offset(0.2, 0.0, 0.0);
  BodyNode* bNode = mSkel->getBodyNode("h_heel_right");
  Marker* m = new Marker("right_foot", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.2, 0.0, 0.0);
  bNode = mSkel->getBodyNode("h_heel_left");
  m = new Marker("left_foot", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.065, -0.3, 0.0);
  bNode = mSkel->getBodyNode("h_thigh_right");
  m = new Marker("right_thigh", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.065, -0.3, 0.0);
  bNode = mSkel->getBodyNode("h_thigh_left");
  m = new Marker("left_thigh", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.0, 0.13);
  bNode = mSkel->getBodyNode("h_pelvis");
  m = new Marker("pelvis_right", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.0, -0.13);
  bNode = mSkel->getBodyNode("h_pelvis");
  m = new Marker("pelvis_left", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.075, 0.1, 0.0);
  bNode = mSkel->getBodyNode("h_abdomen");
  m = new Marker("abdomen", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.18, 0.075);
  bNode = mSkel->getBodyNode("h_head");
  m = new Marker("head_right", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.18, -0.075);
  bNode = mSkel->getBodyNode("h_head");
  m = new Marker("head_left", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.22, 0.0);
  bNode = mSkel->getBodyNode("h_scapula_right");
  m = new Marker("right_scapula", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, 0.22, 0.0);
  bNode = mSkel->getBodyNode("h_scapula_left");
  m = new Marker("left_scapula", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.2, 0.05);
  bNode = mSkel->getBodyNode("h_bicep_right");
  m = new Marker("right_bicep", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.2, -0.05);
  bNode = mSkel->getBodyNode("h_bicep_left");
  m = new Marker("left_bicep", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.1, 0.025);
  bNode = mSkel->getBodyNode("h_hand_right");
  m = new Marker("right_hand", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);

  offset = Vector3d(0.0, -0.1, -0.025);
  bNode = mSkel->getBodyNode("h_hand_left");
  m = new Marker("left_hand", offset, bNode);
  mMarkers.push_back(m);
  bNode->addMarker(m);
}
