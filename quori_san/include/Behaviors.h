#ifndef BEHAVIORS_H_
#define BEHAVIORS_H_

#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "MovementConfigurator.h"
#include "navigation_tools.h"
#include "Listeners.h"

/*
*   controls where and how quickly
*   a robot will move around the map
*/
class Behaviors
{
	protected:
		PositionListener robotPosition;
		point current;
		point goal;
		// whether or not the goal meets the requirements
		bool ok = false;

		/*
		* 	fills the message for action client
		*/
		move_base_msgs::MoveBaseGoal createGoalMessage ()
		{
			move_base_msgs::MoveBaseGoal goalMessage;

			// set up the frame parameters
			goalMessage.target_pose.header.frame_id = "map";
			goalMessage.target_pose.header.stamp = ros::Time::now ();
			// set goal coordinates
			goalMessage.target_pose.pose.position.x =  goal.x;
			goalMessage.target_pose.pose.position.y =  goal.y;
			goalMessage.target_pose.pose.position.z =  0.0;
			goalMessage.target_pose.pose.orientation.x = 0.0;
			goalMessage.target_pose.pose.orientation.y = 0.0;
			goalMessage.target_pose.pose.orientation.z = 0.0;
			goalMessage.target_pose.pose.orientation.w = 0.0;

			return goalMessage;
		}

	public:
		/*
		*   find a goal that fits the behavior
		*/
		virtual point findGoal ()
		{
			return goal;
		}

		/*
		*   send the robot to the goal
		*   from http://edu.gaitech.hk/turtlebot/map-navigation.html
		*/
		bool goToGoal ()
		{
			// define a client for to send goal requests to the move_base server through a SimpleActionClient
			actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> actionClient ("move_base", true);

			// wait for the action server to come up
			while (!actionClient.waitForServer (ros::Duration (5.0)))
			{
				ROS_DEBUG_STREAM ("Waiting for the move_base action server...");
			}

			// send the goal
			actionClient.sendGoal (createGoalMessage ());
			actionClient.waitForResult ();

			return actionClient.getState () == actionlib::SimpleClientGoalState::SUCCEEDED;
		}
};

class Engaging : public Behaviors
{
	public:
		/*
		*   find a goal that fits the behavior
		*/
		point findGoal (std::vector <point> people)
		{
			// enable control of speed limit
			MovementConfigurator movementLimiter;

			// search for a goal
			movementLimiter.setVelocityLimit ('x', 0.5);
			ROS_DEBUG_STREAM ("Set velocity limit to 0.5 ...");
			current = robotPosition.getPosition ();
			int index = people.size ();

			// while there is a person location to test and a valid goal has not been found
			while (index > 0 && !ok)
			{
				// set a new goal halfway between the robot and a person
				goal = current + ((people [index] - current) / 2.0);
				// test the goal
				ok = navigationTools::checkGoal (current, goal);
				// iterate through the people vector
				index -= 1;
			}

			srand (time (NULL));

			// while a valid goal has not been found
			while (!ok)
			{
				ROS_DEBUG_STREAM ("Could not find person to interact with, falling back to wandering...");
				// set a goal at a random location relative to the robot's current location
				goal.x = current.x + (rand () % 100 - 50) / 10;
				goal.y = current.y + (rand () % 100 - 50) / 10;
				// test the goal
				ok = navigationTools::checkGoal (current, goal);
			}

			return goal;
		}
};

class Conservative : public Behaviors
{
	public:
		/*
		*   find a goal that fits the behavior
		*/
		point findGoal (std::vector <point> people)
		{
			MovementConfigurator movementLimiter;

			// search for a goal
			movementLimiter.setVelocityLimit ('x', 0.33);
			ROS_DEBUG_STREAM ("Set velocity limit to 0.33 ...");
			current = robotPosition.getPosition ();
			int index = people.size ();

			// while a valid goal has not been found
			while (index > 0 && !ok)
			{
				// set a new goal, todo: keep a larger distance from people
				goal = current + ((people [index] - current) / 4.0);
				// test the goal
				ok = navigationTools::checkGoal (current, goal);
				// iterate through the people vector
				index -= 1;
			}

			// if goal has not been found
			if (!ok)
			{
				ROS_INFO_STREAM ("Could not find person to interact with, staying in current position...");
				goal = current;
			}

			return goal;
		}
};

class Reserved : public Behaviors
{
	private:
		std::vector <point> reserved;

	public:
		/*
		*   find a goal that fits the behavior
		*/
		point findGoal ()
		{
			point reservedPoint;
			reservedPoint.x = 0.0;
			reservedPoint.y = 0.0;
			reserved.push_back (reservedPoint);

			MovementConfigurator movementLimiter;

			// search for a goal
			movementLimiter.setVelocityLimit ('x', 0.25);
			ROS_DEBUG_STREAM ("Set velocity limit to 0.25 ...");
			current = robotPosition.getPosition ();
			// only go to predefined locations
			int random = rand () % reserved.size ();

			goal = reserved [random];
			ok = navigationTools::checkGoal (current, goal);

			// if goal has not been found
			if (!ok)
			{
				ROS_INFO_STREAM ("Could not go to predetermined location, staying in current position...");
				goal = current;
			}

			return goal;
		}
};

class Stationary : public Behaviors
{
	private:
		// the "kiosk" location for the robot to stay in during stationary behavior
		point stationary;

	public:
		/*
		*   find a goal that fits the behavior
		*/
		point findGoal ()
		{
			stationary.x = 0.0;
			stationary.y = 0.0;

			MovementConfigurator movementLimiter;

			// search for a goal
			movementLimiter.setVelocityLimit ('x', 0.10);
			ROS_DEBUG_STREAM ("Set velocity limit to 0.10 ...");
			current = robotPosition.getPosition ();
			// go to predefined stationary location
			goal = stationary;
			ROS_INFO_STREAM ("Checking goal ...");
			ok = navigationTools::checkGoal (current, goal);

			// if no path to stationary location is available
			if (!ok)
			{
				ROS_INFO_STREAM ("Could not go to stationary location, staying in current position...");
				goal = current;
			}

			return goal;
		}
};

class Welcoming : public Behaviors
{
	private:
		
	public:

};

#endif
