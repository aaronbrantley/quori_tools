#ifndef NAVIGATION_TOOLS_H_
#define NAVIGATION_TOOLS_H_

#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace navigationTools
{
    // makes vector code easier to read
    int x = 0;
    int y = 1;

    /*
    *   https://www.programmersought.com/article/85495009501/
    */
    nav_msgs::GetPlan::Request createRequest (std::vector <double> start, std::vector <double> end)
    {
      // create the request message
      nav_msgs::GetPlan::Request request;

      // set frame for starting position
      request.start.header.frame_id = "map";

      // set coordinates for starting position
      request.start.pose.position.x = start [x];
      request.start.pose.position.y = start [y];

      request.start.pose.orientation.w = 1.0;

      // set frame for ending position
      request.goal.header.frame_id = "map";

      // set coordinates for ending position
      request.goal.pose.position.x = end [x];
      request.goal.pose.position.y = end [y];

      request.goal.pose.orientation.w = 1.0;

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
        // srv.response.plan.poses is the container for storing the results, traversed and taken out
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
    bool checkGoal (std::vector <double> currentCoordinates, std::vector <double> goalCoordinates)
    {
      ros::NodeHandle goalCheckNode;
      ros::ServiceClient planClient = goalCheckNode.serviceClient <nav_msgs::GetPlan> ("move_base_node/make_plan", true);
      nav_msgs::GetPlan planSrv;

      // fill in the request for make_plan service
      createRequest (currentCoordinates, goalCoordinates);

      // if make_plan cannot find a plan
      if (! callPlanningService (planClient, planSrv))
      {
        ROS_DEBUG_STREAM ("goal not ok, no path from planner");

        return false;
      }

      ROS_INFO_STREAM ("potential goal found");

      return true;
    }
};

#endif
