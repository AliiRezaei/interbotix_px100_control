
// include pinocchhio necessary libraries :
#include "pinocchio/fwd.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include <iostream>

// include crba.hpp dependencies :
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/algorithm/check.hpp"
#include "pinocchio/algorithm/crba.hpp"
 
// include ros necessary libraries :
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

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
  
  // // sample a random configuration :
  // Eigen::VectorXd q(6);
  // q << 1.0, 0.5, 0.3, 0.2, 0.4, 0.0;

  // Eigen::VectorXd v(5);
  // v << 0.15, 0.1, 0.05, 0.12, 0.0;

  // Eigen::VectorXd a(5);
  // a << 0.0, 0.0, 0.0, 0.0, 0.0;
 
  Eigen::VectorXd q = pinocchio::neutral(model);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);


  // print configuration :
  std::cout << "q: " << q.transpose() << std::endl;
  std::cout << "v: " << v.transpose() << std::endl;
  std::cout << "a: " << a.transpose() << std::endl;
  std::cout << "model.nv: " << model.nv << std::endl;

  
  // mass matrix :
  pinocchio::crba(model, data, q);
  for(Eigen::Index row = 0; row < model.nv; row++)
  {
    for(Eigen::Index col = 0; col < model.nv; col++)
    {
      if(row > col)
      {
        data.M(row, col) = data.M(col, row);
      }
    }
  }
  std::cout << "\n mass matrix = \n" << data.M << std::endl;



  // coriolis, centrifugal, gravity :
  const Eigen::VectorXd & b = pinocchio::rnea(model, data, q, v, a);
  std::cout << "\n coriolis, centrifugal, gravity = \n" << b.transpose() << std::endl;


  // ROS :
  ros::init(argc, argv, "pinocchio_dynamics_publisher");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("matrix_data", 1);

  ros::Rate rate(10);

  // std_msgs::Float32MultiArray msg;
  // msg.data = {1.0, 2.0, 3.0};
  while(ros::ok())
  {
    std_msgs::Float32MultiArray msg;
    msg.data = {1.0, 2.0};
    pub.publish(msg);
    // ros::spin();
    rate.sleep();
  }
  return 0;
}

