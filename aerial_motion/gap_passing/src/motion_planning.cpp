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

#include <gap_passing/motion_control.h>
#include <gap_passing/motion_planning.h>

/* the StabilityObjective does not work!!!!! */
StabilityObjective::StabilityObjective(ros::NodeHandle nh, ros::NodeHandle nhp, const ompl::base::SpaceInformationPtr& si, boost::shared_ptr<TransformController>  transform_controller, int planning_mode): ompl::base::StateCostIntegralObjective(si, true)
{
  nh_ = nh;
  nhp_ = nhp;
  transform_controller_ = transform_controller;

  planning_mode_  = planning_mode;

  link_num_ = transform_controller_->getRotorNum();

  nhp_.param("robot_type", robot_type_, 0); //0: hydrus; 1: dragon
  nhp_.param("semi_stable_cost", semi_stable_cost_, 0.5);
  nhp_.param("full_stable_cost", full_stable_cost_, 0.0);

}

ompl::base::Cost StabilityObjective::stateCost(const ompl::base::State* state) const
{
  sensor_msgs::JointState joint_state;
  if(robot_type_ == HYDRUS)
    {
      for(int i = 0; i < link_num_ - 1; i++)
        {
          std::stringstream ss;
          ss << i + 1;
          joint_state.name.push_back(std::string("joint") + ss.str());
        }
    }
  if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
    {
      for(int i = 0; i < link_num_ - 1; i++)
        joint_state.position.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    }
  else if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE)
    {
      /* do not need check */
      return ompl::base::Cost(full_stable_cost_);
    }
  else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
    {
      const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(state);
      for(int i = 0; i < link_num_ - 1; i++)
        joint_state.position.push_back(state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i]);
    }

  transform_controller_->kinematics(joint_state);
  /* 1: check the rotor distance from the axes; duplicate with the state validation */

  if(!transform_controller_->distThreCheck())
    return ompl::base::Cost(std::numeric_limits<double>::infinity());

    /* 2: stability check */
  if(!transform_controller_->modelling())
    return ompl::base::Cost(semi_stable_cost_);

  else
    return ompl::base::Cost(full_stable_cost_);
}

MotionPlanning::MotionPlanning(ros::NodeHandle nh, ros::NodeHandle nhp) :nh_(nh), nhp_(nhp)
{
  transform_controller_ = boost::shared_ptr<TransformController>(new TransformController(nh_, nhp_, false));
  link_num_ = transform_controller_->getRotorNum();

  rosParamInit();

  motion_control_ = new MotionControl(nh, nhp, transform_controller_);

  planning_scene_diff_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  if(planning_mode_ != gap_passing::PlanningMode::ONLY_JOINTS_MODE)
    {
      while(planning_scene_diff_pub_.getNumSubscribers() < 1)
        {
          ros::WallDuration sleep_t(0.5);
          sleep_t.sleep();
        }
      ros::Duration sleep_time(1.0);
      sleep_time.sleep();

      robot_model_loader_ = new robot_model_loader::RobotModelLoader("robot_description");
      kinematic_model_ = robot_model_loader_->getModel();
      planning_scene_ = new planning_scene::PlanningScene(kinematic_model_);
      tolerance_ = 0.01;
      acm_ = planning_scene_->getAllowedCollisionMatrix();

      gapEnvInit();
    }

  calculation_time_ = 0;
  best_cost_ = -1;

  if(!play_log_path_) Planning();
  motion_sequence_timer_ = nhp_.createTimer(ros::Duration(1.0 / motion_sequence_rate_), &MotionPlanning::motionSequenceFunc, this);

}

MotionPlanning::~MotionPlanning()
{
  if(planning_mode_ != gap_passing::PlanningMode::ONLY_JOINTS_MODE)
    {
      delete robot_model_loader_;
      delete planning_scene_;
    }

  delete plan_states_;
  delete rrt_start_planner_;
  delete path_length_opt_objective_;
  delete stability_objective_;

  delete motion_control_;
}

void MotionPlanning::motionSequenceFunc(const ros::TimerEvent &e)
{
  static bool first_flag = true;

  static int state_index = 0;
  static std::vector<conf_values> planning_path;

  if(solved_ && ros::ok())
    {
      if(first_flag)
        {
          ROS_WARN("plan size is %d, planning time is %f, motion cost is %f", motion_control_->getPathSize(), motion_control_->getPlanningTime(), motion_control_->getMotionCost());
          float min_var = motion_control_->getMinVar();
          std::cout << "min_var is: "<< min_var;
          int min_var_state_index = motion_control_->getMinVarStateIndex();
          for(int i = 0; i < link_num_ - 1; i++)
            std::cout << " joint" << i+1 << ":" << (motion_control_->getState(min_var_state_index)).state_values[3 + i];
          std::cout << std::endl;

          ROS_WARN("semi stable states %d, ratio: %f", motion_control_->getSemiStableStates(),
                   (float)motion_control_->getSemiStableStates() / motion_control_->getPathSize());

          planning_path.resize(0);
          motion_control_->getPlanningPath(planning_path);

          first_flag = false;
        }
      else
        {
          if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
            {
              sensor_msgs::JointState joint_state_msg;
              joint_state_msg.header.stamp = ros::Time::now();
              if(planning_path[state_index].state_values.size() != 3 + link_num_ -1)
                ROS_ERROR("wrong size of planning joint state and this state");
              joint_state_msg.position.resize(0);
              joint_state_msg.name.resize(0);
              for(int i = 0; i < link_num_ - 1; i ++)
                {
                  std::stringstream joint_no2;
                  joint_no2 << i + 1;
                  joint_state_msg.name.push_back(std::string("joint") + joint_no2.str());
                  joint_state_msg.position.push_back(planning_path[state_index].state_values[3 + i]);
                }
              motion_control_->joint_cmd_pub_.publish(joint_state_msg);
            }
          else
            {
              robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();
              robot_state.setVariablePositions((motion_control_->getState(state_index)).state_values);

              planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
              planning_scene_msg_.is_diff = true;
              planning_scene_diff_pub_.publish(planning_scene_msg_);
            }

          state_index ++;
          if(state_index ==  motion_control_->getPathSize()) state_index = 0;
        }

    }
}

bool  MotionPlanning::isStateValid(const ompl::base::State *state)
{

  std::vector<double> current_state(3 + link_num_ - 1, 0);

  if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
    {
      for(int i = 0; i < link_num_ - 1; i++)
        current_state[3 + i] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
    }
  else if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE || planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE + gap_passing::PlanningMode::ORIGINAL_MODE)
    {
      current_state[0] = state->as<ompl::base::SE2StateSpace::StateType>()->getX();
      current_state[1] = state->as<ompl::base::SE2StateSpace::StateType>()->getY();
      current_state[2] = state->as<ompl::base::SE2StateSpace::StateType>()->getYaw();

      for(int i = 0; i < link_num_ - 1; i++)
        current_state[i + 3] = start_state_[3 + i];

    }
  else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
    {
      const ompl::base::CompoundState* state_tmp = dynamic_cast<const ompl::base::CompoundState*>(state);

      current_state[0] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getX();
      current_state[1] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getY();
      current_state[2] = state_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->getYaw();
      for(int i = 0; i < link_num_ - 1; i++)
        current_state[i + 3] = state_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i];
    }

  //check distance thresold
  if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE || planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
    {
      if(stability_opt_weight_ == 0)
        {//only path length opt, we have to judge the form whethe valid or not
          sensor_msgs::JointState joint_state;
          if(robot_type_ == HYDRUS)
            for(int i = 0; i < link_num_ - 1; i++)
              {
                std::stringstream ss;
                ss << i + 1;
                joint_state.name.push_back(std::string("joint") + ss.str());
                joint_state.position.push_back(current_state[i + 3]);
              }
          transform_controller_->kinematics(joint_state);
          double dist_thre_check = transform_controller_->distThreCheck();

          if(dist_thre_check == 0)
            return false;
          if(!transform_controller_->modelling())
            return false;
        }

      if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE) return true;
    }

  robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();
  //check collision
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  robot_state.setVariablePositions(current_state);
  planning_scene_->checkCollision(collision_request, collision_result, robot_state, acm_);

  if(collision_result.collision) return false;

  return true;
}

void MotionPlanning::gapEnvInit()
{//pub the collision object => gap env
  left_half_corner = tf::Vector3(gap_left_x_, gap_left_y_ , gap_left_width_);
  right_half_corner = tf::Vector3(gap_left_x_ + gap_y_offset_, gap_left_y_ + gap_x_offset_ , gap_right_width_);

  collision_object_.header.frame_id = "world";
  collision_object_.id = "box";
  geometry_msgs::Pose pose1, pose2,pose3,pose4;
  pose1.position.x = left_half_corner.getX() + gap_left_width_ /2;
  pose1.position.y =  (2.5 + gap_x_offset_/2) /2;
  pose1.position.z = 0.0;
  pose1.orientation.w = 1.0;
  pose2.position.x = right_half_corner.getX() + gap_right_width_ /2;
  pose2.position.y = - (2.5 + gap_x_offset_/2) /2;
  pose2.position.z = 0.0;
  pose2.orientation.w = 1.0;

  pose3.position.x = 1.0;
  pose3.position.y = 2.5;
  pose3.position.z = 0.0;
  pose3.orientation.w = 1.0;
  pose4.position.x = 1.0;
  pose4.position.y = -2.5;
  pose4.position.z = 0.0;
  pose4.orientation.w = 1.0;


  shape_msgs::SolidPrimitive primitive1, primitive2, primitive3, primitive4;
  primitive1.type = primitive1.BOX;
  primitive1.dimensions.resize(3);

  primitive1.dimensions[0] = gap_left_width_;
  primitive1.dimensions[1] = 2.5 - gap_x_offset_ / 2;
  primitive1.dimensions[2] = 1;
  collision_object_.primitives.push_back(primitive1);
  collision_object_.primitive_poses.push_back(pose1);
  primitive2.type = primitive2.BOX;
  primitive2.dimensions.resize(3);
  primitive2.dimensions[0] = gap_right_width_;
  primitive2.dimensions[1] = 2.5 - gap_x_offset_ / 2;
  primitive2.dimensions[2] = 1;
  collision_object_.primitives.push_back(primitive2);
  collision_object_.primitive_poses.push_back(pose2);
  primitive3.type = primitive2.BOX;
  primitive3.dimensions.resize(3);
  primitive3.dimensions[0] = 8;
  primitive3.dimensions[1] = 0.6;
  primitive3.dimensions[2] = 1;
  collision_object_.primitives.push_back(primitive3);
  collision_object_.primitive_poses.push_back(pose3);
  primitive4.type = primitive2.BOX;
  primitive4.dimensions.resize(3);
  primitive4.dimensions[0] = 8;
  primitive4.dimensions[1] = 0.6;
  primitive4.dimensions[2] = 1;

  collision_object_.primitives.push_back(primitive4);
  collision_object_.primitive_poses.push_back(pose4);

  collision_object_.operation = collision_object_.ADD;
  planning_scene_->processCollisionObjectMsg(collision_object_);

  //temporarily
  robot_state::RobotState& init_state = planning_scene_->getCurrentStateNonConst();
  init_state.setVariablePositions(start_state_);
  /**/

  planning_scene_->getPlanningSceneMsg(planning_scene_msg_);
  planning_scene_msg_.is_diff = true;
  planning_scene_diff_pub_.publish(planning_scene_msg_);
}

void MotionPlanning::Planning()
{
  //planning
  //x, y
  ompl::base::StateSpacePtr se2(new ompl::base::SE2StateSpace());
  ompl::base::RealVectorBounds motion_bounds(2);
  motion_bounds.low[0] = -2;
  motion_bounds.low[1] = -2.2;
  motion_bounds.high[0] = 5;
  motion_bounds.high[1] = 2.2;
  se2->as<ompl::base::SE2StateSpace>()->setBounds(motion_bounds);
  //joints
  ompl::base::StateSpacePtr r_joints(new ompl::base::RealVectorStateSpace(link_num_ -1));
  ompl::base::RealVectorBounds joint_bounds(link_num_ -1);
  joint_bounds.setLow(-1.58);
  joint_bounds.setHigh(1.58);
  r_joints->as<ompl::base::RealVectorStateSpace>()->setBounds(joint_bounds);

  if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
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
  else if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE)
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
  else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
    {
      config_space_ = se2 + r_joints;
      space_information_  = ompl::base::SpaceInformationPtr(new ompl::base::SpaceInformation(config_space_));
      space_information_->setStateValidityChecker(boost::bind(&MotionPlanning::isStateValid, this, _1));
    }

  /* init state */
  ompl::base::ScopedState<> start(config_space_);
  ompl::base::ScopedState<> goal(config_space_);
  if(planning_mode_ == gap_passing::PlanningMode::ONLY_JOINTS_MODE)
    {
      for(int i = 0; i < link_num_ - 1; i ++)
        {
          start->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = start_state_[3 + i];
          goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = goal_state_[3 + i];
        }
    }
  else if(planning_mode_ == gap_passing::PlanningMode::ONLY_BASE_MODE)
    {
      start->as<ompl::base::SE2StateSpace::StateType>()->setXY(start_state_[0], start_state_[1]);
      start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(start_state_[2]);
      goal->as<ompl::base::SE2StateSpace::StateType>()->setXY(goal_state_[0], goal_state_[1]);
      goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(goal_state_[2]);
    }

  else if(planning_mode_ == gap_passing::PlanningMode::JOINTS_AND_BASE_MODE)
    {
      ompl::base::CompoundState* start_tmp = dynamic_cast<ompl::base::CompoundState*> (start.get());
      start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(start_state_[0], start_state_[1]);
      start_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(start_state_[2]);

      ompl::base::CompoundState* goal_tmp = dynamic_cast<ompl::base::CompoundState*> (goal.get());
      goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setXY(goal_state_[0], goal_state_[1]);
      goal_tmp->as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(goal_state_[2]);

      for(int i = 0; i < link_num_ - 1; i ++)
        {
          start_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = start_state_[3 + i];
          goal_tmp->as<ompl::base::RealVectorStateSpace::StateType>(1)->values[i] = goal_state_[3 + i];
        }

    }

  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(space_information_));
  pdef->setStartAndGoalStates(start, goal);

  //sampler
  space_information_->setValidStateSamplerAllocator(boost::bind(&MotionPlanning::allocValidStateSampler, this, _1));

  //optimation objective
  path_length_opt_objective_ = new ompl::base::PathLengthOptimizationObjective(space_information_);
  path_length_opt_objective_->setCostThreshold(onlyJointPathLimit());
  //deubg
  std::cout << "path length opt cost thre is "<<  path_length_opt_objective_->getCostThreshold() << std::endl;

  stability_objective_ = new StabilityObjective(nh_, nhp_, space_information_, transform_controller_, planning_mode_);
  stability_objective_->setCostThreshold(ompl::base::Cost(stability_cost_thre_));
  std::cout << "stability_objective  opt cost thre is "<<  stability_objective_->getCostThreshold() << std::endl;

  ompl::base::OptimizationObjectivePtr lengthObj(path_length_opt_objective_);
  ompl::base::OptimizationObjectivePtr stabilityObj(stability_objective_);

  ompl::base::PlannerPtr planner;
  if(ompl_mode_ == RRT_START_MODE)
    {
      //original for optimization
      if(length_opt_weight_ == 0)
        {
          pdef->setOptimizationObjective(stabilityObj);
          ROS_WARN("only stability optimazation");
        }
      else if(stability_opt_weight_ == 0)
        {
          pdef->setOptimizationObjective(lengthObj);
          ROS_WARN("only path lengyh optimazation");

        }
      else
        {
          //reset the cost thresold of pathlength
          path_length_opt_objective_->setCostThreshold(ompl::base::Cost(std::numeric_limits<double>::infinity()));
          pdef->setOptimizationObjective(length_opt_weight_* lengthObj + stability_opt_weight_ * stabilityObj);
          ROS_WARN("both path lengyh and stability optimazation");
        }

      rrt_start_planner_ = new ompl::geometric::RRTstar(space_information_);
      planner = ompl::base::PlannerPtr(rrt_start_planner_);
    }

  planner->setProblemDefinition(pdef);
  planner->setup();
  space_information_->printSettings(std::cout);

  solved_ = false;
  ros::Time start_time = ros::Time::now();

  ompl::base::PlannerStatus solved = planner->solve(solving_time_limit_);

  if (solved)
    {
      calculation_time_ = ros::Time::now().toSec() - start_time.toSec();
      path_ = pdef->getSolutionPath();
      std::cout << "Found solution:" << std::endl;
      path_->print(std::cout);

      if(ompl_mode_ == RRT_START_MODE)
        {
          std::cout << "iteration is "<< rrt_start_planner_->numIterations() << "best cost is " << rrt_start_planner_->bestCost()  << std::endl;
          std::stringstream ss;
          ss << rrt_start_planner_->bestCost();
          ss >> best_cost_;
        }

      //visualztion
      plan_states_ = new ompl::base::StateStorage(config_space_);
      int index = (int)(std::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getStateCount());
      ROS_ERROR("index: %d", index);
      for(int i = 0; i < index; i++)
        {
          ompl::base::State *state1;
          ompl::base::State *state2;

          if(i == 0)
            state1 = start.get();
          else
            state1 = std::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getState(i - 1);

          state2 = std::static_pointer_cast<ompl::geometric::PathGeometric>(path_)->getState(i);


          plan_states_->addState(state1);
          int nd = config_space_->validSegmentCount(state1, state2);
          if (nd > 1)
            {
              ompl::base::State *interpolated_state = space_information_->allocState();
              for (int j = 1 ; j < nd ; ++j)
                {
                  config_space_->interpolate(state1, state2, (double)j / (double)nd, interpolated_state);
                  plan_states_->addState(interpolated_state);

                }
            }
          plan_states_->addState(state2);
        }

      motion_control_->planStoring(plan_states_, planning_mode_, start_state_, goal_state_, best_cost_, calculation_time_);

      solved_ = true;
    }
  else
    std::cout << "No solution found" << std::endl;

}

void MotionPlanning::rosParamInit()
{
  nhp_.param("robot_type", robot_type_, 0); //0: hydrus; 1: dragon

  nhp_.param("gap_left_x", gap_left_x_, 1.0);
  nhp_.param("gap_left_y", gap_left_y_, 0.3);
  nhp_.param("gap_x_offset", gap_x_offset_, 0.6); //minus: overlap
  nhp_.param("gap_y_offset", gap_y_offset_, 0.0); //minus: overlap
  nhp_.param("gap_left_width", gap_left_width_, 0.3); //minus: bandwidth
  nhp_.param("gap_right_width", gap_right_width_, 0.3); //minus: bandwidth

  //nhp_.param("coefficient_rate", coefficient_rate_, 0.035);
  nhp_.param("ompl_mode", ompl_mode_, 0); //RRT_START_MODE
  nhp_.param("planning_mode", planning_mode_, 0); //ONLY_JOINTS_MODE
  nhp_.param("motion_sequence_rate", motion_sequence_rate_, 10.0);

  start_state_.resize(3 + link_num_ - 1);
  nhp_.param("start_state_x", start_state_[0], 0.0);
  nhp_.param("start_state_y", start_state_[1], 0.5);
  nhp_.param("start_state_theta", start_state_[2], 0.785);
  goal_state_.resize(3 + link_num_ - 1);
  nhp_.param("goal_state_x", goal_state_[0], 3.0);
  nhp_.param("goal_state_y", goal_state_[1], 0.5);
  nhp_.param("goal_state_theta", goal_state_[2], 0.785);

  for(int i = 0; i < link_num_ - 1; i++)
    {
      std::stringstream joint_no;
      joint_no << i + 1;

      nhp_.param(std::string("start_state_joint") + joint_no.str(), start_state_[3 + i], 0.0);
      nhp_.param(std::string("goal_state_joint") + joint_no.str(), goal_state_[3 + i], 0.0);
    }

  nhp_.param("real_robot_move_base", real_robot_move_base_, false);

  nhp_.param("state_validity_check_res", state_validity_check_res_, 0.03);
  nhp_.param("valid_segment_count_factor", valid_segment_count_factor_,20);

  nhp_.param("solving_time_limit", solving_time_limit_, 3600.0);


  nhp_.param("length_opt_weight", length_opt_weight_, 1.0);
  nhp_.param("stability_opt_weight", stability_opt_weight_, 0.0);

  nhp_.param("stability_cost_thre", stability_cost_thre_, 100000000.0);

  nhp_.param("play_log_path", play_log_path_, false);
  if(play_log_path_) solved_ = true;
}

ompl::base::Cost MotionPlanning::onlyJointPathLimit()
{
  double cost = 0.1;
  for(int i = 0; i < link_num_ - 1; i ++)
    cost += fabs(start_state_[3 + i] - goal_state_[3 + i]);
  return ompl::base::Cost(cost);
}