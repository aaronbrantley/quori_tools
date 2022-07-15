#ifndef BEHAVIORS_H_
#define BEHAVIORS_H_

#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/GetPlan.h>

#include "MovementConfigurator.h"

/*
*   controls where and how quickly
*   a robot will move around the map
*/
class Behaviors
{
  protected:
    // makes coordinate vector code easier to read
    int x = 0;
    int y = 1;

    /*
    *   sets up the request to create a path from
    *   startCoordinates to endCoordinates
    *   from https://www.programmersought.com/article/85495009501/
    */
    void fillPathRequest (nav_msgs::GetPlan::Request & request, std::vector <double> startCoordinates, std::vector <double> endCoordinates)
    {
      // set frame for starting position
      request.start.header.frame_id = "map";

      // set coordinates for starting position
      request.start.pose.position.x = startCoordinates [x];
      request.start.pose.position.y = startCoordinates [y];

      request.start.pose.orientation.w = 1.0;

      // set frame for ending position
      request.goal.header.frame_id = "map";

      // set coordinates for ending position
      request.goal.pose.position.x = endCoordinates [x];
      request.goal.pose.position.y = endCoordinates [y];

      request.goal.pose.orientation.w = 1.0;

      // from getplan service documentaion:
      // If the goal is obstructed, how many meters the planner can relax the constraint in x and y before failing.
      request.tolerance = 0.0;
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
    bool checkGoal (std::vector <double> currentCoordinates, std::vector <double> goalCoordinates, double locationThreshold)
    {
      ros::NodeHandle goalCheckNode;
      ros::ServiceClient planClient = goalCheckNode.serviceClient <nav_msgs::GetPlan> ("move_base/make_plan", true);
      nav_msgs::GetPlan planSrv;

      // fill in the request for make_plan service
      fillPathRequest (planSrv.request, currentCoordinates, goalCoordinates);

      // if make_plan cannot find a plan
      if (!callPlanningService (planClient, planSrv))
      {
        ROS_DEBUG_STREAM ("goal not ok, no path from planner");
        return false;
      }

      ROS_INFO_STREAM ("potential goal found");

      return true;
    }

  public:
    /*
    *   find a goal that fits the behavior
    */
    virtual std::vector <double> findGoal (std::vector <double> currentCoordinates, std::vector <std::vector <double>> peopleLocations)
    {
      // set a navigation goal

      return {0, 0};
    }

    /*
    *   send the robot to the goal
    *   from http://edu.gaitech.hk/turtlebot/map-navigation.html
    */
    bool goToGoal (std::vector <double> goalCoordinates)
    {
      // define a client for to send goal requests to the move_base server through a SimpleActionClient
      actionlib::SimpleActionClient <move_base_msgs::MoveBaseAction> ac ("move_base", true);

      // wait for the action server to come up
      while (!ac.waitForServer (ros::Duration (5.0)))
      {
        ROS_DEBUG_STREAM ("waiting for the move_base action server to come up");
      }

      move_base_msgs::MoveBaseGoal goal;

      // set up the frame parameters
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now ();

      // set goal coordinates
      goal.target_pose.pose.position.x =  goalCoordinates [x];
      goal.target_pose.pose.position.y =  goalCoordinates [y];
      goal.target_pose.pose.position.z =  0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 0.0;
      goal.target_pose.pose.orientation.w = 1.0;

      //ROS_INFO_STREAM ("goal: (" << goalCoordinates [x] << ", " << goalCoordinates [y] << ")");

      // send the goal
      ac.sendGoal (goal);
      ac.waitForResult ();

      if (ac.getState () == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
       //ROS_INFO_STREAM ("robot reached the destination");
       return true;
      }

      else
      {
       //ROS_WARN_STREAM ("robot did not reach the destination");
       return false;
      }
    }
};

class Engaging : public Behaviors
{
  public:
    /*
    *   find a goal that fits the behavior
    */
    std::vector <double> findGoal (std::vector <double> currentCoordinates, std::vector <std::vector <double>> peopleLocations)
    {
      // enable control of speed limit
      MovementConfigurator movementLimiter;

      // stores goals to be tested
      std::vector <double> potentialGoal;
      // whether or not the goal meets the requirements
      bool goalIsOk = false;

      // search for a goal
      ROS_INFO_STREAM ("engaging behavior");

      // set speed limit
      movementLimiter.setVelocityLimit ('x', 0.45);

      int index = peopleLocations.size ();

      // while there is a person location to test and a valid goal has not been found
      while (index > 0 && !goalIsOk)
      {
        // forget the previous invalid goal
        potentialGoal.clear ();

        // set a new goal halfway between the robot and a person
        potentialGoal.push_back ((currentCoordinates [x] + peopleLocations.at (index) [x]) / 2);
        potentialGoal.push_back ((currentCoordinates [y] + peopleLocations.at (index) [y]) / 2);

        // test the goal
        goalIsOk = checkGoal (currentCoordinates, potentialGoal, 0.5);

        // iterate through the people vector backwards (its sorted from least reliable to most)
        index -= 1;
      }

      // if goal between a person has not been found
      if (!goalIsOk)
      {
        ROS_INFO_STREAM ("could not find person to interact with, falling back to wandering");
        srand (time (NULL));
        // wander randomly to indicate less serious tone attitude
        do
        {
          // forget the previous invalid goal
          potentialGoal.clear ();

          // set a goal at a random location relative to the robot's current location
          potentialGoal.push_back (currentCoordinates [x] + (rand () % 100 - 50) / 10);
          potentialGoal.push_back (currentCoordinates [y] + (rand () % 100 - 50) / 10);

          // test the goal
          goalIsOk = checkGoal (currentCoordinates, potentialGoal, 0.5);
        }
        // while a valid goal has not been found
        while (!goalIsOk);
      }

      return potentialGoal;
    }

    /*
    *   find a goal that fits the behavior
    */
    std::vector <double> findGoal (std::vector <double> currentCoordinates)
    {
      // enable control of speed limit
      MovementConfigurator movementLimiter;

      // stores goals to be tested
      std::vector <double> potentialGoal;
      // whether or not the goal meets the requirements
      bool goalIsOk = false;

      // search for a goal
      ROS_INFO_STREAM ("engaging behavior (no people detected - wandering)");

      // set speed limit
      movementLimiter.setVelocityLimit ('x', 0.45);

      srand (time (NULL));

      // wander randomly to indicate less serious tone attitude
      // while a valid goal has not been found
      while (!goalIsOk)
      {
        // forget the previous invalid goal
        potentialGoal.clear ();

        // set a goal at a random location relative to the robot's current location
        potentialGoal.push_back (currentCoordinates [x] + (rand () % 100 - 50) / 10);
        potentialGoal.push_back (currentCoordinates [y] + (rand () % 100 - 50) / 10);

        // test the goal
        goalIsOk = checkGoal (currentCoordinates, potentialGoal, 0.5);
      }


      return potentialGoal;
    }
};

class Conservative : public Behaviors
{
  public:
    /*
    *   find a goal that fits the behavior
    */
    std::vector <double> findGoal (std::vector <double> currentCoordinates, std::vector <std::vector <double>> peopleLocations)
    {
      MovementConfigurator movementLimiter;

      // stores goals to be tested
      std::vector <double> potentialGoal;
      // whether or not the goal meets the requirements
      bool goalIsOk = false;

      // search for a goal
      ROS_INFO_STREAM ("conservative behavior");

      movementLimiter.setVelocityLimit ('x', 0.35);

      int index = peopleLocations.size ();

      // while a valid goal has not been found
      while (index > 0 && !goalIsOk)
      {
        // forget the previous invalid goal
        potentialGoal.clear ();

        // set a new goal, todo: keep a larger distance from people
        potentialGoal.push_back ((currentCoordinates [x] + peopleLocations.at (index) [x]) / 2);
        potentialGoal.push_back ((currentCoordinates [y] + peopleLocations.at (index) [y]) / 2);

        // test the goal
        goalIsOk = checkGoal (currentCoordinates, potentialGoal, 1.0);

        // iterate through the people vector backwards (its sorted from least reliable to most)
        index -= 1;
      }

      // if goal has not been found
      if (!goalIsOk)
      {
        ROS_INFO_STREAM ("could not find person to interact with, staying in current position");

        // stay where you are
        potentialGoal = currentCoordinates;
        ROS_INFO_STREAM ("potentialGoal set");
      }

      return potentialGoal;
    }
};

class Reserved : public Behaviors
{
  public:
    /*
    *   find a goal that fits the behavior
    */
    std::vector <double> findGoal (std::vector <double> currentCoordinates, std::vector <std::vector <double>> peopleLocations)
    {
      MovementConfigurator movementLimiter;

      // a set of predetermined locations for the robot to go to during reserved behavior
      std::vector <std::vector <double>> reservedLocations = {{0, 0}, {0, 0}};

      // stores goals to be tested
      std::vector <double> potentialGoal;
      // whether or not the goal meets the requirements
      bool goalIsOk = false;

      // search for a goal
      ROS_INFO_STREAM ("reserved behavior");

      movementLimiter.setVelocityLimit ('x', 0.25);

      bool alreadyAtReservedLocation = false;
      double locationThreshold = 0.33;

      // only go to predetermined locations

      for (int index = 0; index < reservedLocations.size (); index += 1)
      {
        if (abs (currentCoordinates [x] - reservedLocations.at (index) [x]) < locationThreshold && abs (currentCoordinates [y] - reservedLocations.at (index) [y]) < locationThreshold)
        {
          alreadyAtReservedLocation = true;
        }
      }

      if (alreadyAtReservedLocation)
      {
        ROS_INFO_STREAM ("already at reserved location, staying in current position");
        goalIsOk = true;
        potentialGoal = currentCoordinates;
        ROS_INFO_STREAM ("potentialGoal set");
      }

      else
      {
        int randomReservedLocation = rand () % 3;

        potentialGoal.clear ();

        potentialGoal.push_back (reservedLocations.at (randomReservedLocation) [x]);
        potentialGoal.push_back (reservedLocations.at (randomReservedLocation) [y]);

        goalIsOk = checkGoal (currentCoordinates, potentialGoal, locationThreshold);
      }

      // if goal has not been found
      if (!goalIsOk)
      {
        ROS_INFO_STREAM ("could not go to predetermined location, staying in current position");
        // stay where you are
        potentialGoal = currentCoordinates;
        ROS_INFO_STREAM ("potentialGoal set");
      }

      return potentialGoal;
    }
};

class Stationary : public Behaviors
{
  public:
    /*
    *   find a goal that fits the behavior
    */
    std::vector <double> findGoal (std::vector <double> currentCoordinates, std::vector <std::vector <double>> peopleLocations)
    {
      MovementConfigurator movementLimiter;

      // the "kiosk" location for the robot to stay in during stationary behavior
      std::vector <double> stationaryLocation = {-3.35937, -3.2264};

      // stores goals to be tested
      std::vector <double> potentialGoal;
      // whether or not the goal meets the requirements
      bool goalIsOk = false;

      // search for a goal
      ROS_INFO_STREAM ("stationary behavior");

      ROS_INFO_STREAM ("setting velocity limit to 0.15 ...");
      movementLimiter.setVelocityLimit ('x', 0.15);

      // go to designated stationary location

      potentialGoal = stationaryLocation;

      ROS_INFO_STREAM ("checking goal ...");
      goalIsOk = checkGoal (currentCoordinates, potentialGoal, 0.33);

      // if no path to stationary location is available
      if (!goalIsOk)
      {
        ROS_INFO_STREAM ("could not go to stationary location, staying in current position");
        // stay where you are
        potentialGoal = currentCoordinates;
        ROS_INFO_STREAM ("potentialGoal set");
      }

      return potentialGoal;
    }
};

#endif
