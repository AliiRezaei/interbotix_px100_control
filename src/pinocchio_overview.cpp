
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
#include "sensor_msgs/JointState.h"

// define model URDF directory path :
#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "/home/ali/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_descriptions/urdf/"
#endif


void joint_state_callback(sensor_msgs::JointState msg)
{
  ROS_INFO("msg : []");
}



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
  Eigen::VectorXd q = pinocchio::neutral(model);
  q(0) = 1.0;
  q(1) = 1.5;
  q(2) = 0.3;
  q(3) = 0.5;
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);


  // print configuration :
  std::cout << "q: " << q.transpose() << std::endl;
  std::cout << "v: " << v.transpose() << std::endl;
  std::cout << "a: " << a.transpose() << std::endl;
  std::cout << "model.nv: " << model.nv << std::endl;


  // ROS :
  ros::init(argc, argv, "pinocchio_dynamics_publisher");
  ros::NodeHandle nh;

  ros::Publisher mass_matrix_pub = nh.advertise<std_msgs::Float32MultiArray>("mass_matrix_data", 1);
  ros::Subscriber joint_state_sub = nh.subscribe("/px100/joint_states", 1, joint_state_callback);

  ros::Rate pub_rate(10);
  
  std_msgs::Float32MultiArray mass_matrix_msg;
  mass_matrix_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mass_matrix_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
  mass_matrix_msg.layout.dim[0].label = "height";
  mass_matrix_msg.layout.dim[0].size = model.nv;
  mass_matrix_msg.layout.dim[0].stride = model.nv * model.nv;
  mass_matrix_msg.layout.dim[1].label = "width";
  mass_matrix_msg.layout.dim[1].size = model.nv;
  mass_matrix_msg.layout.dim[1].stride = model.nv;
  mass_matrix_msg.layout.data_offset = 0;
  mass_matrix_msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  int count = 0;

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
      mass_matrix_msg.data[count++] = data.M(row, col);
    }
  }
  std::cout << "\n mass matrix = \n" << data.M << std::endl;



  // coriolis, centrifugal, gravity :
  const Eigen::VectorXd & b = pinocchio::rnea(model, data, q, v, a);
  std::cout << "\n coriolis, centrifugal, gravity = \n" << b.transpose() << std::endl;

  while(ros::ok())
  {
    mass_matrix_pub.publish(mass_matrix_msg);
    pub_rate.sleep();
  }
  return 0;
}

