#ifndef NAVIGATION_TOOLS_H_
#define NAVIGATION_TOOLS_H_

#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "point.h"

namespace navigationTools
{
	/*
	*   https://www.programmersought.com/article/85495009501/
	*/
	nav_msgs::GetPlan::Request createRequest (point start, point end)
	{
		// create the request message
		nav_msgs::GetPlan::Request request;

		// set frame for starting position
		request.start.header.frame_id = "map";
		// set coordinates for starting position
		request.start.pose.position.x = start.x;
		request.start.pose.position.y = start.y;
		request.start.pose.orientation.w = 0.0;
		// set frame for ending position
		request.goal.header.frame_id = "map";
		// set coordinates for ending position
		request.goal.pose.position.x = end.x;
		request.goal.pose.position.y = end.y;
		request.goal.pose.orientation.w = 0.0;
		// from getplan service documentaion:
		// If the goal is obstructed, how many meters the planner can relax the constraint in x and y before failing.
		request.tolerance = 0.0;

		return request;
	}

	/*
	*   attempt to make a navigation plan
	*   from https://www.programmersought.com/article/85495009501/
	*/
	bool callPlanningService (ros::ServiceClient & serviceClient, nav_msgs::GetPlan & serviceMessage)
	{
		// perform the actual path planner call
		// execute the actual path planner
		if (serviceClient.call (serviceMessage))
		{
			// serviceMessage.response.plan.poses is the container for storing the results, traversed and taken out
			if (!serviceMessage.response.plan.poses.empty ())
			{
				//ROS_DEBUG_STREAM ("make_plan success");

				return true;
			}
		}

		return false;
	}

	/*
	*   check if the goal will fit the behavior
	*/
	bool checkGoal (point current, point goal)
	{
		ros::NodeHandle goalCheck;
		ros::ServiceClient planClient = goalCheck.serviceClient <nav_msgs::GetPlan> ("move_base_node/make_plan", true);
		nav_msgs::GetPlan planMessage;

		// fill in the request for make_plan service
		createRequest (current, goal);

		// if make_plan cannot find a plan
		if (!callPlanningService (planClient, planMessage))
		{
			ROS_DEBUG_STREAM ("No path from planner for goal (" << current.x << ", " << current.y << ")");

			return false;
		}

		ROS_INFO_STREAM ("Path from planner for goal (" << current.x << ", " << current.y << ")");

		return true;
	}
};

#endif
