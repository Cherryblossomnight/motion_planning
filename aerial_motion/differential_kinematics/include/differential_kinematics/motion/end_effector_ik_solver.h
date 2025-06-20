// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef END_EFFECTOR_IK_SOLVER_H_
#define END_EFFECTOR_IK_SOLVER_H_

/* ros */
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <aerial_robot_msgs/FlightNav.h>
#include <aerial_robot_msgs/PoseControlPid.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <spinal/DesireCoord.h>

/* robot model */
#include <hydrus/hydrus_tilted_robot_model.h>
#include <dragon/model/hydrus_like_robot_model.h> // TODO: change to full vectoring model
#include <dragon/dragon_navigation.h>
#include <aerial_motion_planning_msgs/multilink_state.h>
#include <aerial_motion_planning_msgs/continuous_path_generator.h>
/* path search */
#include <pluginlib/class_loader.h>
#include <differential_kinematics/motion/end_effector_ik_solver_core.h>
/* utils */
#include <tf/LinearMath/Transform.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <boost/algorithm/clamp.hpp>

class EndEffectorIKSolver{
public:
  EndEffectorIKSolver(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~EndEffectorIKSolver(){}

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Subscriber plan_flag_sub_;
  ros::Subscriber move_flag_sub_;
  ros::Subscriber return_flag_sub_;
  ros::Subscriber robot_baselink_odom_sub_;
  ros::Subscriber robot_joint_states_sub_;
  ros::Subscriber control_terms_sub_;

  ros::Publisher joints_ctrl_pub_;
  ros::Publisher flight_nav_pub_;
  ros::Publisher se3_roll_pitch_nav_pub_;
  ros::Publisher end_effector_pos_pub_;
  ros::Publisher debug_pub_; // TODO: need?

  bool debug_verbose_;
  bool headless_;

  /* state converngece */
  double joint_thresh_;
  double pos_thresh_;
  double rot_thresh_;

  /* robot joint limitation */
  std::vector<double> angle_min_vec_;
  std::vector<double> angle_max_vec_;

  /* discrete path */
  int discrete_path_search_method_type_;
  bool discrete_path_debug_flag_;
  int motion_type_;
  std::string robot_type_;

  /* navigation */
  ros::Timer navigate_timer_;
  double move_start_time_;
  bool real_machine_connect_;
  bool move_flag_;
  bool plan_flag_;
  bool return_flag_;
  double controller_freq_;
  double return_delay_;

  nav_msgs::Odometry robot_baselink_odom_;
  aerial_robot_msgs::PoseControlPid control_terms_;
  KDL::JntArray joint_state_;

  int state_index_;
  std_msgs::ColorRGBA desired_state_color_;
  double start_return_time_;
  double max_joint_vel_; //debug


  /* robot model */
  boost::shared_ptr<HydrusRobotModel> robot_model_ptr_;

  /* discrete path search */

  /* end-effector ik solver */
  boost::shared_ptr<EndEffectorIKSolverCore> end_effector_ik_solver_core_;

  void rosParamInit();
  virtual void process(const ros::TimerEvent& event);
  void pathSearch();
  void pathExecute();
  void pathExecute(const std::vector<MultilinkState>& discrete_path, boost::shared_ptr<ContinuousPathGenerator> continuous_path);

  /* robot real state */
  void robotOdomCallback(const nav_msgs::OdometryConstPtr& msg);
  void robotJointStatesCallback(const sensor_msgs::JointStateConstPtr& joint_msg);
  void controlTermsCallback(const aerial_robot_msgs::PoseControlPidConstPtr& control_msg);


  virtual void reset();
  void startNavigate();

};

#endif
