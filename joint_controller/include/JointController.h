#ifndef	JOINT_CONTROLLER_H_
#define JOINT_CONTROLLER_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient <control_msgs::FollowJointTrajectoryAction> actionClient;

/*
* 	from http://library.isr.ist.utl.pt/docs/roswiki/pr2_controllers(2f)Tutorials(2f)Moving(20)the(20)arm(20)using(20)the(20)Joint(20)Trajectory(20)Action.html#Creating_the_node
*/
class JointController
{
	private:
		std::vector <std::string> joint_names;
		float leftShoulderPitch, leftShoulderRoll, rightShoulderPitch, rightShoulderRoll, waistPitch;
		actionClient * client;

	protected:
		void sendCommand ()
		{
			trajectory_msgs::JointTrajectoryPoint point;
			trajectory_msgs::JointTrajectory trajectory;
			control_msgs::FollowJointTrajectoryGoal goal;

			point.positions.push_back (leftShoulderPitch);
			point.positions.push_back (leftShoulderRoll);
			point.positions.push_back (rightShoulderPitch);
			point.positions.push_back (rightShoulderRoll);
			point.positions.push_back (waistPitch);
			point.velocities.empty ();
			point.accelerations.empty ();
			point.time_from_start = ros::Duration (0.02);

			trajectory.header.stamp = ros::Time::now () + ros::Duration (0.01);
			trajectory.joint_names = joint_names;
			trajectory.points.push_back (point);

			goal.trajectory = trajectory;

			client -> sendGoal (goal);
		}

	public:
		JointController ()
		{
			client = new actionClient ("quori/joint_trajectory_controller/follow_joint_trajectory");

			while (!client -> waitForServer (ros::Duration (5.0)))
			{
				ROS_INFO ("Waiting for joint_trajectory_controller action server");
			}

			joint_names.push_back ("l_shoulder_pitch");
			joint_names.push_back ("l_shoulder_roll");
			joint_names.push_back ("r_shoulder_pitch");
			joint_names.push_back ("r_shoulder_roll");
			joint_names.push_back ("waist_pitch");

			leftShoulderPitch = 0.0;
			leftShoulderRoll = 0.0;
			rightShoulderPitch = 0.0;
			rightShoulderRoll = 0.0;
			waistPitch = 0.0;
		}

		void setLeftShoulderPitch (float value)
		{
			leftShoulderPitch = value;
		}

		void setLeftShoulderRoll (float value)
		{
			leftShoulderRoll = value;
		}

		void setRightShoulderPitch (float value)
		{
			rightShoulderPitch = value;
		}

		void setRightShoulderRoll (float value)
		{
			rightShoulderRoll = value;
		}

		void setWaistPitch (float value)
		{
			waistPitch = value;
		}

		actionlib::SimpleClientGoalState createGoal ()
		{
			sendCommand ();

			while (!client -> getState ().isDone () && ros::ok ())
			{
				continue;
			}

			return client -> getState ();
		}
};

#endif
