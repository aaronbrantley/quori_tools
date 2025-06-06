#ifndef	JOINT_CONTROLLER_H_
#define JOINT_CONTROLLER_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "joint_values.h"

// from http://library.isr.ist.utl.pt/docs/roswiki/pr2_controllers(2f)Tutorials(2f)Moving(20)the(20)arm(20)using(20)the(20)Joint(20)Trajectory(20)Action.html#Creating_the_node
typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> actionClient;

/*
* 	sends a goal to quori's joint controller
*/
class JointController
{
	private:
		// the list of quori's controllable joints
		std::vector <std::string> joint_names;
		// the values associated with each joint
		joint_values values;
		// action client for sending goals to the controller
		actionClient * client;

	protected:
		/*
		* 	send a goal to the controllers
		* 	given the current joint values
		*/
		void sendCommand ()
		{
			trajectory_msgs::JointTrajectoryPoint point;
			trajectory_msgs::JointTrajectory trajectory;
			control_msgs::FollowJointTrajectoryGoal goal;

			point.positions.push_back (values.leftShoulderPitch);
			point.positions.push_back (values.leftShoulderRoll);
			point.positions.push_back (values.rightShoulderPitch);
			point.positions.push_back (values.rightShoulderRoll);
			point.positions.push_back (values.waistPitch);
			point.velocities.empty ();
			point.accelerations.empty ();
			point.time_from_start = ros::Duration (0.02);

			trajectory.header.stamp = ros::Time::now () + ros::Duration (0.01);
			trajectory.joint_names = joint_names;
			trajectory.points.push_back (point);

			goal.trajectory = trajectory;

			client -> sendGoal (goal);
		}

		/*
		* 	adjust the transition speed for the joint controller
		*/
		void sendCommand (float speed)
		{
			trajectory_msgs::JointTrajectoryPoint point;
			trajectory_msgs::JointTrajectory trajectory;
			control_msgs::FollowJointTrajectoryGoal goal;

			point.positions.push_back (values.leftShoulderPitch);
			point.positions.push_back (values.leftShoulderRoll);
			point.positions.push_back (values.rightShoulderPitch);
			point.positions.push_back (values.rightShoulderRoll);
			point.positions.push_back (values.waistPitch);
			point.velocities.empty ();
			point.accelerations.empty ();
			point.time_from_start = ros::Duration (speed);

			trajectory.header.stamp = ros::Time::now () + ros::Duration (speed / 2);
			trajectory.joint_names = joint_names;
			trajectory.points.push_back (point);

			goal.trajectory = trajectory;

			client -> sendGoal (goal);
		}

	public:
		/*
		* 	default constructor
		* 	startup action client
		* 	assign joint names
		*/
		JointController ()
		{
			client = new actionClient ("quori/joint_trajectory_controller/follow_joint_trajectory");

			while (!client -> waitForServer (ros::Duration (5.0)))
			{
				ROS_INFO ("Waiting for joint_trajectory_controller action server");
			}

			joint_names = values.jointNames;
		}

		void setLeftShoulderPitch (float value)
		{
			values.leftShoulderPitch = value;
		}

		void setLeftShoulderRoll (float value)
		{
			values.leftShoulderRoll = value;
		}

		void setRightShoulderPitch (float value)
		{
			values.rightShoulderPitch = value;
		}

		void setRightShoulderRoll (float value)
		{
			values.rightShoulderRoll = value;
		}

		void setWaistPitch (float value)
		{
			values.waistPitch = value;
		}

		/*
		* 	send a goal to the action server
		* 	and wait for the controller to finish
		*/
		bool createGoal ()
		{
			sendCommand ();

			while (!client -> getState ().isDone () && ros::ok ())
			{
				continue;
			}

			return client -> getState () == actionlib::SimpleClientGoalState::SUCCEEDED;
		}

		/*
		* 	adjust the transition speed for the joint controller
		*/
		bool createGoal (float speed)
		{
			sendCommand (speed);

			while (!client -> getState ().isDone () && ros::ok ())
			{
				continue;
			}

			return client -> getState () == actionlib::SimpleClientGoalState::SUCCEEDED;
		}
};

#endif
