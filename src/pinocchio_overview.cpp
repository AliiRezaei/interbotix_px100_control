
// include pinocchhio necessary libraries :
#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
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
  pinocchio::urdf::buildModel(urdf_filename, model);
  std::cout << "model name: " << model.name << std::endl;
  
  // create data required by the algorithms :
  Data data(model);
  
  // sample a random configuration :
  // Eigen::VectorXd q = randomConfiguration(model);
  Eigen::VectorXd q(6);
  q << 1.0, 0.5, 0.3, 0.2, 0.4, 0.0;

  Eigen::VectorXd v(5);
  v << 0.15, 0.1, 0.05, 0.12, 0.0;

  Eigen::VectorXd a(5);
  a << 0.0, 0.0, 0.0, 0.0, 0.0;

  // std::cout << "q: " << q.transpose() << std::endl;
  // Eigen::VectorXd q = pinocchio::neutral(model);
  // Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nv+1);
  // Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

  // Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);
 
  // print configuration :
  std::cout << "q: " << q.transpose() << std::endl;
  std::cout << "v: " << v.transpose() << std::endl;
  std::cout << "a: " << a.transpose() << std::endl;

  // mass matrix :
  pinocchio::computeAllTerms(model, data, q, v);
  // const Eigen::MatrixXd & M = pinocchio::crba(model, data, q);
  std::cout << "mass matrix = " << data.M << std::endl;

  // coriolis, centrifugal, gravity :
  const Eigen::VectorXd & b = pinocchio::rnea(model, data, q, v, a);
  std::cout << "Coriolis, centrifugal, gravity = " << b.transpose() << std::endl;

  // std::cout << "M matrix is = " << data.M << std::endl;
  // std::cout << "G matrix is = " << pinocchio::computeGeneralizedGravity(model, data, q) << std::endl;


  // // perform the forward kinematics over the kinematic tree :
  // forwardKinematics(model,data,q);
 
  // // print out the placement of each joint of the kinematic tree
  // for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
  //   std::cout << std::setw(24) << std::left
  //             << model.names[joint_id] << ": "
  //             << std::fixed << std::setprecision(2)
  //             << data.oMi[joint_id].translation().transpose()
  //             << std::endl;
}

