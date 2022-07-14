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


    // std
    std::string nodeName;
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
      // tf
      tf::StampedTransform transformObject;
      tf::TransformListener transformListener;

      try
      {
        transformListener.lookupTransform ("map", "quori/base_link", ros::Time (0), transformObject);
      }
      catch (tf::TransformException & exception)
      {
        ROS_ERROR_STREAM (exception.what ());
      }

      position.clear ();

      position.push_back (transformObject.getOrigin ().x ());
      position.push_back (transformObject.getOrigin ().y ());
    }

  public:
    std::vector <double> getPose ()
    {
      nodeName = "pose_listener";

      initializeRos (nodeName);



      ROS_DEBUG_STREAM ("pose transform return value: (" << position [x] << ", " << position [y] << ")");

      return position;
    }
};

#endif
