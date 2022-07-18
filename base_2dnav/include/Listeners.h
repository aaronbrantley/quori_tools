#ifndef LISTENERS_H_
#define LISTENERS_H_

#include <ros/ros.h>

#include <tf/transform_listener.h>

/*
*   finds the position of the robot in the map
*   from the transform between the robot frame
*   and the map frame
*/
class PoseListener
{
  private:
    // ros
    ros::Time latest;
    ros::Duration waitTime;

    // tf
    tf::StampedTransform transformObject;

    // std
    std::string nodeName;
    std::string mapFrame;
    std::string robotFrame;
    std::vector <double> position;

    // misc
    int x = 0;
    int y = 1;

  protected:
    /*
    *   use roscpp functions here
    *   after ros::init
    */
    void initializeRos (std::string name)
    {
      int argc = 0;
      char ** argv = nullptr;

      ros::init (argc, argv, name);

      tf::TransformListener transformListener;

      try
      {
        transformListener.waitForTransform (mapFrame, robotFrame, latest, waitTime);
        transformListener.lookupTransform (mapFrame, robotFrame, latest, transformObject);
      }
      catch (tf::TransformException & exception)
      {
        ROS_ERROR_STREAM (exception.what ());
      }
    }

  public:
    /*
    *   store the position of the robot
    *   into a vector
    *   position [0] = x coordinate
    *   position [1] = y coordinate
    */
    std::vector <double> getPose ()
    {
      latest = ros::Time (0);
      waitTime = ros::Duration (1);

      nodeName = "pose_listener";
      mapFrame = "map";
      robotFrame = "quori/base_link";

      initializeRos (nodeName);

      position.clear ();

      position.push_back (transformObject.getOrigin ().x ());
      position.push_back (transformObject.getOrigin ().y ());

      ROS_DEBUG_STREAM ("pose transform return value: (" << position [x] << ", " << position [y] << ")");

      return position;
    }
};

#endif
