
// include piinocchhio necessary libraries :
#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include <iostream>
 
// include ros necessary libraries :
#include "ros/ros.h"

// define model URDF directory path :
#ifndef PINOCCHIO_MODEL_DIR
 #define PINOCCHIO_MODEL_DIR "/home/ali/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf/"
#endif
 
int main(int argc, char ** argv)
{
  using namespace pinocchio;
  
  // URDF file namere  :
  const std::string urdf_filename = (argc<=1) ? PINOCCHIO_MODEL_DIR + std::string("px100_modified.urdf.xacro") : argv[1];

  // load the URDF model :
  Model model;
  pinocchio::urdf::buildModel(urdf_filename,model);
  std::cout << "model name: " << model.name << std::endl;
  
  // create data required by the algorithms :
  Data data(model);
  
  // sample a random configuration :
  Eigen::VectorXd q = randomConfiguration(model);
  // std::cout << "q: " << q.transpose() << std::endl;
  // Eigen::VectorXd q = pinocchio::neutral(model);
  // Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nv+1);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  v(1) = 1; v(2) = 1; //v(2, 0) = 1; v(3, 0) = 1;
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
 
  // print configuration :
  std::cout << "q: " << q.transpose() << std::endl;
  std::cout << "v: " << v.transpose() << std::endl;
  std::cout << "a: " << a.transpose() << std::endl;

  // inverse dynamics :
  const Eigen::VectorXd & tau = pinocchio::rnea(model,data,q,v,a);
  std::cout << "tau = " << tau.transpose() << std::endl;

  std::cout << "mass matrix is = " << data.M << std::endl;


  // perform the forward kinematics over the kinematic tree :
  forwardKinematics(model,data,q);
 
  // print out the placement of each joint of the kinematic tree
  for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left
              << model.names[joint_id] << ": "
              << std::fixed << std::setprecision(2)
              << data.oMi[joint_id].translation().transpose()
              << std::endl;
}

