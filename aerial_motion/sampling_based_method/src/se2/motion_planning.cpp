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

#include <sampling_based_method/se2/motion_planning.h>

namespace
{
  bool motion_sequence_first_flag = true;
  int state_index = 0;

  bool odom_flag = false;
}

namespace sampling_base
{
  namespace se2
  {
    MotionPlanning::MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp, boost::shared_ptr<TransformController> transform_controller):
      nh_(nh), nhp_(nhp), transform_controller_(transform_controller),
      real_odom_flag_(false), path_(0), calculation_time_(0),
      best_cost_(-1), min_var_(1e6),
      planning_mode_(sampling_based_method::PlanningMode::ONLY_JOINTS_MODE),
      motion_type_(sampling_based_method::PlanningMode::SE2)
    {
      /* ros pub/sub and service */
      planning_scene_diff_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

      /* init */
      robotInit();
      rosParamInit();
    }

    void MotionPlanning::robotInit()
    {
      sensor_msgs::JointState actuator_state;
      for (auto itr: transform_controller_->getActuatorMap())
        {
          actuator_state.name.push_back(itr.first);
          actuator_state.position.push_back(0);
        }

      transform_controller_->setActuatorJointMap(actuator_state); // get joint index from get actuator map
      joint_num_ = transform_controller_->getActuatorJointMap().size();

      start_state_.setActuatorState(actuator_state);
      goal_state_.setActuatorState(actuator_state);
    }

    void MotionPlanning::motionSequence()
    {
      /* moveit */
      planning_scene_->setCurrentState(getStateConst(state_index).getVisualizeRobotStateConst());
      planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
      planning_scene_msg_.is_diff = true;
      planning_scene_diff_pub_.publish(planning_scene_msg_);
    }

    bool  MotionPlanning::isStateValid(const ompl::base::State *state)
    {
      MultilinkState current_state = start_state_;
      geometry_msgs::Pose root_pose;

      if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE)
        {
          int index = 0;
          for(auto itr : transform_controller_->getActuatorJointMap())
            current_state.setActuatorState(itr, state->as<ompl::base::RealVectorStateSpace::StateType>()->values[index++]);
        }
      else if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_BASE_MODE)
        {
          root_pose.position.x = state->as<ompl::base::SE2StateSpace::StateType>()->getX();
          root_pose.position.y = state->as<ompl::base::SE2StateSpace::StateType>()->getY();
          root_pose.orientation = tf::createQuaternionMsgFromYaw(state->as<ompl::base::SE2StateSpace::StateType>()->getYaw());
        }
      else if(planning_mode_ == sampling_based_method::PlanningMode::JOINTS_AND_BASE_MODE)
        {
          const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(state);
          root_pose.position.x = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
          root_pose.position.y = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
          root_pose.orientation = tf::createQuaternionMsgFromYaw( state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw());

          int index = 0;
          for(auto itr : transform_controller_->getActuatorJointMap())
            current_state.setActuatorState(itr, state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[index++]);
        }

      current_state.setRootPose(root_pose);

      //check distance thresold
      if(planning_mode_ == sampling_based_method::PlanningMode::JOINTS_AND_BASE_MODE || planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE)
        {
          transform_controller_->forwardKinematics(current_state.getActuatorStateNonConst());

          if(!transform_controller_->stabilityMarginCheck()) return false;
          if(!transform_controller_->modelling()) return false;

          if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE) return true;
        }

      //check collision
      collision_detection::CollisionRequest collision_request;
      collision_detection::CollisionResult collision_result;
      planning_scene_->setCurrentState(current_state.getVisualizeRobotStateConst());
      //planning_scene_->checkCollision(collision_request, collision_result, planning_scene_->getCurrentState(), acm_);
      planning_scene_->checkCollision(collision_request, collision_result);

      if(collision_result.collision) return false;

      return true;
    }

    void MotionPlanning::sceneInit()
    {
      while(planning_scene_diff_pub_.getNumSubscribers() < 1)
        {
          ros::WallDuration sleep_t(0.5);
          sleep_t.sleep();
        }
      ros::Duration sleep_time(1.0);
      sleep_time.sleep();

      robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
      planning_scene_ = boost::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(robot_model_loader.getModel()));
      acm_ = planning_scene_->getAllowedCollisionMatrix();

      gapEnvInit();

      planning_scene_->setCurrentState(start_state_.getVisualizeRobotStateConst());
      planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
      planning_scene_msg_.is_diff = true;
      planning_scene_diff_pub_.publish(planning_scene_msg_);

      //check collision
      collision_detection::CollisionRequest collision_request;
      collision_detection::CollisionResult collision_result;
      //planning_scene_->checkCollision(collision_request, collision_result, planning_scene_->getCurrentState(), acm_);
      planning_scene_->checkCollision(collision_request, collision_result);

      if(collision_result.collision) ROS_ERROR("collsion with init state");
    }

    void MotionPlanning::gapEnvInit()
    {
      if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE) return;

      //pub the collision object => gap env
      left_half_corner = tf::Vector3(gap_left_x_, gap_left_y_ , gap_left_width_);
      right_half_corner = tf::Vector3(gap_left_x_ + gap_x_offset_, gap_left_y_ + gap_x_offset_ , gap_right_width_);

      moveit_msgs::CollisionObject collision_object;
      collision_object.header.frame_id = "world";
      collision_object.id = "box";
      geometry_msgs::Pose wall_pose;
      wall_pose.orientation.w = 1.0;
      shape_msgs::SolidPrimitive wall_primitive;
      wall_primitive.type = wall_primitive.BOX;
      wall_primitive.dimensions.resize(3);

      wall_pose.position.x = left_half_corner.getX() + gap_left_width_ /2;
      wall_pose.position.y =  (2.5 + gap_y_offset_/2) /2;
      wall_primitive.dimensions[0] = gap_left_width_;
      wall_primitive.dimensions[1] = 2.5 - gap_y_offset_ / 2;
      wall_primitive.dimensions[2] = 1;
      collision_object.primitives.push_back(wall_primitive);
      collision_object.primitive_poses.push_back(wall_pose);

      wall_pose.position.x = right_half_corner.getX() + gap_right_width_ /2;
      wall_pose.position.y = - (2.5 + gap_y_offset_/2) /2;
      wall_pose.orientation.w = 1.0;
      wall_primitive.dimensions[0] = gap_right_width_;
      wall_primitive.dimensions[1] = 2.5 - gap_y_offset_ / 2;
      wall_primitive.dimensions[2] = 1;
      collision_object.primitives.push_back(wall_primitive);
      collision_object.primitive_poses.push_back(wall_pose);

      wall_pose.position.x = 1.0;
      wall_pose.position.y = 2.5;
      wall_pose.orientation.w = 1.0;
      wall_primitive.dimensions[0] = 8;
      wall_primitive.dimensions[1] = 0.6;
      wall_primitive.dimensions[2] = 1;
      collision_object.primitives.push_back(wall_primitive);
      collision_object.primitive_poses.push_back(wall_pose);

      wall_pose.position.x = 1.0;
      wall_pose.position.y = -2.5;
      wall_pose.orientation.w = 1.0;
      wall_primitive.dimensions[0] = 8;
      wall_primitive.dimensions[1] = 0.6;
      wall_primitive.dimensions[2] = 1;
      collision_object.primitives.push_back(wall_primitive);
      collision_object.primitive_poses.push_back(wall_pose);

      collision_object.operation = collision_object.ADD;
      planning_scene_->processCollisionObjectMsg(collision_object);

      /* set the correct base link ( which is not root_link = link1), to be suitable for the control system */
      transform_controller_->setBaselink(base_link_);
    }

    void MotionPlanning::planInit()
    {
      //reset
      path_.clear();
      assert(path_.size() == 0);
      motion_sequence_first_flag = true; // motion sequence

      //set root link as the baselink for the planning
      transform_controller_->setBaselink(std::string("link1"));

      //planning
      //x, y
      ompl::base::StateSpacePtr se2(new ompl::base::SE2StateSpace());
      ompl::base::RealVectorBounds motion_bounds(2);
      motion_bounds.low[0] = x_low_bound_;
      motion_bounds.low[1] = y_low_bound_;
      motion_bounds.high[0] = x_high_bound_;
      motion_bounds.high[1] = y_high_bound_;
      se2->as<ompl::base::SE2StateSpace>()->setBounds(motion_bounds);
      //joints
      ompl::base::StateSpacePtr r_joints(new ompl::base::RealVectorStateSpace(joint_num_));
      ompl::base::RealVectorBounds joint_bounds(joint_num_);
      joint_bounds.setLow(joint_low_bound_);
      joint_bounds.setHigh(joint_high_bound_);
      r_joints->as<ompl::base::RealVectorStateSpace>()->setBounds(joint_bounds);

      if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE)
        {
          config_space_ = r_joints;
          config_space_->as<ompl::base::RealVectorStateSpace>()->setValidSegmentCountFactor(valid_segment_count_factor_);
          space_information_  = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(config_space_));
          space_information_->setup();
          space_information_->setStateValidityChecker(boost::bind(&MotionPlanning::isStateValid, this, _1));
          space_information_->setStateValidityCheckingResolution(state_validity_check_res_);
          space_information_->setMotionValidator(ompl::base::MotionValidatorPtr(new ompl::base::DiscreteMotionValidator(space_information_)));
          space_information_->setup();

        }
      else if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_BASE_MODE)
        {
          config_space_ = se2;
          config_space_->as<ompl::base::SE2StateSpace>()->setValidSegmentCountFactor(valid_segment_count_factor_);
          space_information_  = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(config_space_));
          space_information_->setup();
          space_information_->setStateValidityChecker(boost::bind(&MotionPlanning::isStateValid, this, _1));
          space_information_->setStateValidityCheckingResolution(state_validity_check_res_);
          space_information_->setMotionValidator(ompl::base::MotionValidatorPtr(new ompl::base::DiscreteMotionValidator(space_information_)));
          space_information_->setup();
        }
      else if(planning_mode_ == sampling_based_method::PlanningMode::JOINTS_AND_BASE_MODE)
        {
          config_space_ = se2 + r_joints;
          config_space_->as<ompl::base::CompoundStateSpace>()->setSubspaceWeight(1, 0.001);
          space_information_  = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(config_space_));
          space_information_->setStateValidityChecker(boost::bind(&MotionPlanning::isStateValid, this, _1));
        }

      /* init state */
      ompl::base::ScopedState<> start(config_space_);
      ompl::base::ScopedState<> goal(config_space_);
      if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE)
        {
          for(int i = 0; i < joint_num_; i ++)
            {
              start->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state_.getActuatorStateConst().position.at(transform_controller_->getActuatorJointMap().at(i));
              goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state_.getActuatorStateConst().position.at(transform_controller_->getActuatorJointMap().at(i));
            }
        }
      else if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_BASE_MODE)
        {
          start->as<ompl::base::SE2StateSpace::StateType>()->setXY(start_state_.getRootPoseConst().position.x, start_state_.getRootPoseConst().position.y);
          start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(tf::getYaw(start_state_.getRootPoseConst().orientation));
          goal->as<ompl::base::SE2StateSpace::StateType>()->setXY(goal_state_.getRootPoseConst().position.x, goal_state_.getRootPoseConst().position.y);
          goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(tf::getYaw(goal_state_.getRootPoseConst().orientation));
        }
      else if(planning_mode_ == sampling_based_method::PlanningMode::JOINTS_AND_BASE_MODE)
        {
          ompl::base::CompoundState* start_tmp = dynamic_cast<ompl::base::CompoundState*> (start.get());
          start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(start_state_.getRootPoseConst().position.x, start_state_.getRootPoseConst().position.y);
          start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(tf::getYaw(start_state_.getRootPoseConst().orientation));
          ompl::base::CompoundState* goal_tmp = dynamic_cast<ompl::base::CompoundState*> (goal.get());
          goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(goal_state_.getRootPoseConst().position.x, goal_state_.getRootPoseConst().position.y);
          goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(tf::getYaw(goal_state_.getRootPoseConst().orientation));
          for(int i = 0; i < joint_num_; i ++)
            {
              start_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = start_state_.getActuatorStateConst().position.at(transform_controller_->getActuatorJointMap().at(i));
              goal_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = goal_state_.getActuatorStateConst().position.at(transform_controller_->getActuatorJointMap().at(i));
            }
        }

      pdef_ = ompl::base::ProblemDefinitionPtr(new ompl::base::ProblemDefinition(space_information_));
      pdef_->setStartAndGoalStates(start, goal);
    }

    void MotionPlanning::plan()
    {
      if(headless_)
        {
          ROS_ERROR("sampling base method: please set headless to false, if do planning");
          return;
        }

      sceneInit();
      planInit();

      //sampler
      space_information_->setValidStateSamplerAllocator(boost::bind(&MotionPlanning::allocValidStateSampler, this, _1));

      //optimation objective
      ompl::base::OptimizationObjectivePtr length_obj(new ompl::base::PathLengthOptimizationObjective(space_information_));
      length_obj->setCostThreshold(onlyJointPathLimit());
      std::cout << "path length opt cost thre is "<<  length_obj->getCostThreshold() << std::endl;

      ompl::base::PlannerPtr planner;
      if(ompl_mode_ == RRT_START_MODE)
        {
          pdef_->setOptimizationObjective(length_obj);
          planner = ompl::base::PlannerPtr(new ompl::geometric::RRTstar(space_information_));
        }

      planner->setProblemDefinition(pdef_);
      planner->setup();
      space_information_->printSettings(std::cout);

      ros::Time start_time = ros::Time::now();

      if (planner->solve(solving_time_limit_))
        {
          calculation_time_ = ros::Time::now().toSec() - start_time.toSec();
          ompl::base::PathPtr ompl_result = pdef_->getSolutionPath();
          std::cout << "Found solution:" << std::endl;
          ompl_result->print(std::cout);

          if(ompl_mode_ == RRT_START_MODE)
            {
              std::cout << "iteration is "<< std::static_pointer_cast<ompl::geometric::RRTstar>(planner)->numIterations() << "best cost is " << std::static_pointer_cast<ompl::geometric::RRTstar>(planner)->bestCost()  << std::endl;
              std::stringstream ss;
              ss << std::static_pointer_cast<ompl::geometric::RRTstar>(planner)->bestCost();
              ss >> best_cost_;
            }

          ROS_ERROR("state count: %d", (int)(std::static_pointer_cast<ompl::geometric::PathGeometric>(ompl_result)->getStateCount()));
          for(int i = 1; i < (int)(std::static_pointer_cast<ompl::geometric::PathGeometric>(ompl_result)->getStateCount()); i++)
            {
              ompl::base::State *state1;
              ompl::base::State *state2;

              state1 = std::static_pointer_cast<ompl::geometric::PathGeometric>(ompl_result)->getState(i - 1);

              state2 = std::static_pointer_cast<ompl::geometric::PathGeometric>(ompl_result)->getState(i);

              addState(state1);
              int nd = config_space_->validSegmentCount(state1, state2);
              if (nd > 1)
                {
                  ompl::base::State *interpolated_state = space_information_->allocState();
                  for (int j = 1 ; j < nd ; ++j)
                    {
                      config_space_->interpolate(state1, state2, (double)j / (double)nd, interpolated_state);
                      addState(interpolated_state);

                    }
                }
              addState(state2);
            }

          /* log data */
          ROS_WARN("plan size is %d, planning time is %f, motion cost is %f, min var is %f, min var state index: %d", getPathSize(), getPlanningTime(), getMotionCost(), getMinVar(), getMinVarStateIndex());

          if(save_path_flag_) savePath();
        }
      else
        std::cout << "No solution found" << std::endl;
    }

    void MotionPlanning::rosParamInit()
    {
      nhp_.param("headless", headless_, true);
      nhp_.param("save_path_flag", save_path_flag_, true);
      nhp_.param("file_name", file_name_, std::string("planning_log.txt"));

      nhp_.param("gap_left_x", gap_left_x_, 1.0);
      nhp_.param("gap_left_y", gap_left_y_, 0.3);
      nhp_.param("gap_x_offset", gap_x_offset_, 0.6); //minus: overlap
      nhp_.param("gap_y_offset", gap_y_offset_, 0.0); //minus: overlap
      nhp_.param("gap_left_width", gap_left_width_, 0.3); //minus: bandwidth
      nhp_.param("gap_right_width", gap_right_width_, 0.3); //minus: bandwidth

      nhp_.param("x_low_bound", x_low_bound_, -2.0);
      nhp_.param("x_high_bound", x_high_bound_, 5.0);
      nhp_.param("y_low_bound", y_low_bound_, -2.2);
      nhp_.param("y_high_bound", y_high_bound_, 2.2);
      nhp_.param("joint_low_bound", joint_low_bound_, -1.58);
      nhp_.param("joint_high_bound", joint_high_bound_, 1.58);

      nhp_.param("ompl_mode", ompl_mode_, 0); //RRT_START_MODE
      nhp_.param("planning_mode", planning_mode_, 0); //ONLY_JOINTS_MODE

      nhp_.param("baselink", base_link_, std::string("link1"));
      nhp_.param("motion_type", motion_type_, 0); //SE2
      ROS_ERROR("motion planning: %s", base_link_.c_str());

      geometry_msgs::Pose pose;
      double yaw;
      nhp_.param("start_state_x", pose.position.x, 0.0);
      nhp_.param("start_state_y", pose.position.y, 0.5);
      nhp_.param("start_state_theta", yaw, 0.785);
      pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      start_state_.setRootPose(pose);

      nhp_.param("goal_state_x", pose.position.x, 0.0);
      nhp_.param("goal_state_y", pose.position.y, 0.5);
      nhp_.param("goal_state_theta", yaw, 0.785);
      pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      goal_state_.setRootPose(pose);

      for(int i = 0; i < joint_num_; i++)
        {
          int joint_no = transform_controller_->getActuatorJointMap().at(i);
          nhp_.param(std::string("start_") + start_state_.getActuatorStateNonConst().name.at(joint_no), start_state_.getActuatorStateNonConst().position.at(joint_no), 0.0);
          nhp_.param(std::string("goal_") + goal_state_.getActuatorStateNonConst().name.at(joint_no), goal_state_.getActuatorStateNonConst().position.at(joint_no), 0.0);
        }

      /* debug
         for(int i = 0; i < start_state_.getActuatorStateConst().name.size(); i++)
         {
         ROS_WARN("%s: start: %f, goal: %f", start_state_.getActuatorStateConst().name.at(i).c_str(),
         start_state_.getActuatorStateConst().position.at(i),
         goal_state_.getActuatorStateConst().position.at(i));
         }
      */

      nhp_.param("file_state_offset_x", file_state_offset_x_, 0.0);
      nhp_.param("file_state_offset_y", file_state_offset_y_, 0.0);
      nhp_.param("file_state_offset_z", file_state_offset_z_, 0.0);

      nhp_.param("state_validity_check_res", state_validity_check_res_, 0.03);
      nhp_.param("valid_segment_count_factor", valid_segment_count_factor_,20);

      nhp_.param("solving_time_limit", solving_time_limit_, 3600.0);
      nhp_.param("length_cost_thre", length_cost_thre_, 0.0);
    }

    ompl::base::Cost MotionPlanning::onlyJointPathLimit()
    {
      if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE ||
         planning_mode_ == sampling_based_method::PlanningMode::JOINTS_AND_BASE_MODE)
        {
          if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE) length_cost_thre_ = 0.1;
          for(int i = 0; i < joint_num_; i ++)
            length_cost_thre_ += fabs(start_state_.getActuatorStateConst().position.at(i) - goal_state_.getActuatorStateConst().position.at(i));
        }
      return ompl::base::Cost(length_cost_thre_);
    }

    void MotionPlanning::addState(ompl::base::State *ompl_state)
    {
      MultilinkState new_state = start_state_;
      geometry_msgs::Pose root_pose;

      if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE || planning_mode_ == sampling_based_method::PlanningMode::JOINTS_AND_BASE_MODE)
        {
          if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_JOINTS_MODE)
            {
              int index = 0;
              for(auto itr : transform_controller_->getActuatorJointMap())
                new_state.setActuatorState(itr, ompl_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[index++]);
            }
          else if(planning_mode_ == sampling_based_method::PlanningMode::JOINTS_AND_BASE_MODE)
            {
              const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(ompl_state);

              root_pose.position.x = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
              root_pose.position.y = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
              root_pose.orientation = tf::createQuaternionMsgFromYaw( state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw());
              new_state.setRootPose(root_pose);

              int index = 0;
              for(auto itr : transform_controller_->getActuatorJointMap())
                new_state.setActuatorState(itr, state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[index++]);
            }

          transform_controller_->forwardKinematics(new_state.getActuatorStateNonConst());
          transform_controller_->stabilityMarginCheck();

          if(transform_controller_->getStabilityMargin() < min_var_)
            {
              min_var_ = transform_controller_->getStabilityMargin() ;
              min_var_state_ = path_.size();
            }
        }
      else if(planning_mode_ == sampling_based_method::PlanningMode::ONLY_BASE_MODE)
        {
          root_pose.position.x = ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getX();
          root_pose.position.y = ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getY();
          root_pose.orientation = tf::createQuaternionMsgFromYaw(ompl_state->as<ompl::base::SE2StateSpace::StateType>()->getYaw());
          new_state.setRootPose(root_pose);
        }

      addState(new_state);
    }

    void MotionPlanning::savePath()
    {
      std::ofstream ofs;
      ofs.open(file_name_);
      ofs << "start_state_: ";
      /* root pose */
      for (int i = 0; i < 7; i++) ofs << " " << start_state_.getRootActuatorStateConst().at(i);
      /* joint state */
      for (int j = 0; j < joint_num_; j++)
        ofs << " " << start_state_.getActuatorStateConst().position.at(transform_controller_->getActuatorJointMap().at(j));
      ofs << std::endl;
      ofs << "goal_state_: ";
      /* root pose */
      for (int i = 0; i < 7; i++) ofs << " " << goal_state_.getRootActuatorStateConst().at(i);
      /* joint state */
      for (int j = 0; j < joint_num_; j++)
        ofs << " " << goal_state_.getActuatorStateConst().position.at(transform_controller_->getActuatorJointMap().at(j));
      ofs << std::endl;

      ofs << "states: " << path_.size()  << std::endl;
      ofs << "planning_mode: " << planning_mode_ << std::endl;
      ofs << "planning_time: " << calculation_time_ << std::endl;
      ofs << "motion_cost: " << best_cost_ << std::endl;
      ofs << "minimum_var: " << min_var_ << std::endl;
      ofs << "minimum_var_state_entry: " << min_var_state_  << std::endl;

      for(auto k = 0; k < (int)path_.size();  k++)
        {
          ofs << "state" << k << ": ";
          /* root pose */
          for (int i = 0; i < 7; i++) ofs << " " << path_.at(k).getRootActuatorStateConst().at(i);
          /* joint state */
          for (int j = 0; j < joint_num_; j++)
            ofs << " " << path_.at(k).getActuatorStateConst().position.at(transform_controller_->getActuatorJointMap().at(j));

          ofs << std::endl;
        }
      ofs << "end"  << std::endl;
      ofs.close();
    }

    void MotionPlanning::loadPath()
    {

      std::ifstream ifs(file_name_.c_str());

      if(ifs.fail())
        {
          ROS_ERROR("File do not exist");
          path_.clear();
          return;
        }

      int state_list;
      std::stringstream ss[11];
      std::string str;
      std::string header;
      std::vector<double> state_vec(7 + transform_controller_->getActuatorMap().size());
      //1 start and goal state
      std::getline(ifs, str);
      ss[0].str(str);
      ss[0] >> header;
      /* root pose */
      for (int i = 0; i < 7; i++) ss[0] >> state_vec.at(i);
      /* joint state */
      for (int i = 0; i < joint_num_; i++) ss[0] >> state_vec.at(7 + transform_controller_->getActuatorJointMap().at(i));

      state_vec.at(0) += file_state_offset_x_;
      state_vec.at(1) += file_state_offset_y_;
      state_vec.at(2) += file_state_offset_z_;

      start_state_.setRootActuatorState(state_vec);

      std::getline(ifs, str);
      ss[1].str(str);
      ss[1] >> header;
      /* root pose */
      for (int i = 0; i < 7; i++) ss[1] >> state_vec.at(i);
      /* joint state */
      for (int i = 0; i < joint_num_; i++) ss[1] >> state_vec.at(7 + transform_controller_->getActuatorJointMap().at(i));

      state_vec.at(0) += file_state_offset_x_;
      state_vec.at(1) += file_state_offset_y_;
      state_vec.at(2) += file_state_offset_z_;

      goal_state_.setRootActuatorState(state_vec);

      std::getline(ifs, str);
      ss[2].str(str);
      ss[2] >> header >> state_list;
      std::cout << header << state_list <<std::endl;
      std::getline(ifs, str);
      ss[3].str(str);
      ss[3] >> header >> planning_mode_;
      std::cout << header << planning_mode_ <<std::endl;
      std::getline(ifs, str);
      ss[4].str(str);
      ss[4] >> header >> calculation_time_;
      std::cout << header << calculation_time_ <<std::endl;
      std::getline(ifs, str);
      ss[5].str(str);
      ss[5] >> header >> best_cost_;
      std::cout << header << best_cost_ <<std::endl;
      std::getline(ifs, str);
      ss[6].str(str);
      ss[6] >> header >> min_var_;
      std::cout << header << min_var_ << std::endl;
      std::getline(ifs, str);
      ss[7].str(str);
      ss[7] >> header >> min_var_state_;
      std::cout << header << min_var_state_  <<std::endl;

      for(int k = 0; k < state_list;  k++)
        {
          std::stringstream ss_tmp;

          std::getline(ifs, str);
          ss_tmp.str(str);
          ss_tmp >> header;
          /* root pose */
          for (int i = 0; i < 7; i++) ss_tmp >> state_vec.at(i);
          /* joint state */
          for (int i = 0; i < joint_num_; i++) ss_tmp >> state_vec.at(7 + transform_controller_->getActuatorJointMap().at(i));

          state_vec.at(0) += file_state_offset_x_;
          state_vec.at(1) += file_state_offset_y_;
          state_vec.at(2) += file_state_offset_z_;

          MultilinkState state_tmp;
          state_tmp.setActuatorState(start_state_.getActuatorStateConst()); // init joint state
          state_tmp.setRootActuatorState(state_vec); //init root and joint state
          state_tmp.targetRootPose2TargetBaselinkPose(transform_controller_);
          path_.push_back(state_tmp);
        }

      /* debug */
      /*
      ROS_WARN("start pos: [%f, %f, %f], goal pos: [%f, %f, %f]",
               start_state_.getRootPoseNonConst().position.x,
               start_state_.getRootPoseNonConst().position.y,
               start_state_.getRootPoseNonConst().position.z,
               goal_state_.getRootPoseNonConst().position.x,
               goal_state_.getRootPoseNonConst().position.y,
               goal_state_.getRootPoseNonConst().position.z);

      for(int i = 0; i < start_state_.getActuatorStateConst().name.size(); i++)
        {
          ROS_WARN("%s: start: %f, goal: %f", start_state_.getActuatorStateConst().name.at(i).c_str(),
                   start_state_.getActuatorStateConst().position.at(i),
                   goal_state_.getActuatorStateConst().position.at(i));
        }
      */
      if(!headless_) sceneInit();

      ROS_WARN("plan size is %d, planning time is %f, motion cost is %f, min var is %f, min var state index: %d", getPathSize(), getPlanningTime(), getMotionCost(), getMinVar(), getMinVarStateIndex());

    }
#if 0
    void MotionPlanning::visualizeRobotState(const MultilinkState& state)
    {
      planning_scene_->setCurrentState(state.getVisualizeRobotStateConst());
      planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
      planning_scene_msg_.is_diff = true;
      planning_scene_diff_pub_.publish(planning_scene_msg_);
    }
#endif

    bool MotionPlanning::checkCollision(const MultilinkState& state)
    {
      collision_detection::CollisionRequest collision_request;
      collision_detection::CollisionResult collision_result;
      planning_scene_->setCurrentState(state.getVisualizeRobotStateConst());
      planning_scene_->checkCollision(collision_request, collision_result); //TODO acm_
      if(collision_result.collision) ROS_WARN("Robot collision with env");
    }
  }
}