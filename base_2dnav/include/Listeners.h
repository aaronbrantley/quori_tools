#ifndef LISTENERS_H_
#define LISTENERS_H_

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

// class for getting amcl pose
class PoseListener
{
  private:
    // ros
    ros::Subscriber amclSub;

    // std
    std::string nodeName;
    std::vector <double> poseAMCL;

    // primitive
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

      ros::NodeHandle listenNode;

      amclSub = listenNode.subscribe ("amcl_pose", 1, & PoseListener::amclCallback, this);
    }

    /*
    *   get the latest amcl_pose message and store its data in a vector
    */
    void amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & amclMessage)
    {
      // clear the vector before pushing back
      // this will stop the vector from becoming larger than 2 doubles (x and y coordinates)
      poseAMCL.clear ();

      // insert x coordinate
      poseAMCL.push_back (amclMessage -> pose.pose.position.x);

      // insert y coordinate
      poseAMCL.push_back (amclMessage -> pose.pose.position.y);

      ROS_DEBUG_STREAM ("amcl callback value: (" << amclMessage -> pose.pose.position.x << ", " << amclMessage -> pose.pose.position.y << ")");
    }

  public:
    std::vector <double> getPose ()
    {
      nodeName = "pose_listener";

      initializeRos (nodeName);

      ROS_DEBUG_STREAM ("amcl return value: (" << poseAMCL.at (x) << ", " << poseAMCL.at (y) << ")");

      return poseAMCL;
    }
};

#endif
