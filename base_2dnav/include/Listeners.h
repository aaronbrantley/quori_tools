#ifndef LISTENERS_H_
#define LISTENERS_H_

#include <ros/ros.h>

#include <tf/transform_listener.h>

/*
*   finds the position of the robot in the map
*/
class PoseListener
{
  private:
    // ros
    ros::Time now;
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
    *   allow calling roscpp functions
    */
    void initializeRos (std::string name)
    {
      int argc = 0;
      char ** argv = nullptr;

      ros::init (argc, argv, name);

      tf::TransformListener transformListener;

      try
      {
        transformListener.waitForTransform (mapFrame, robotFrame, now, waitTime);
        transformListener.lookupTransform (mapFrame, robotFrame, now, transformObject);
      }
      catch (tf::TransformException & exception)
      {
        ROS_ERROR_STREAM (exception.what ());
      }
    }

  public:
    std::vector <double> getPose ()
    {
      now = ros::Time (0);
      waitTime = ros::Duration (5.0);

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
