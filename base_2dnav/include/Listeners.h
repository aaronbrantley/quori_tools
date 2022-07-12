#ifndef LISTENERS_H_
#define LISTENERS_H_

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <people_msgs/People.h>
#include <nav_msgs/OccupancyGrid.h>

// class for getting amcl pose
class PoseListener
{
  private:
    ros::NodeHandle poseNode;
    //ROS_DEBUG ("created nodehandle poseNode");

    // subscribe to amcl pose to get estimated robot position
    ros::Subscriber amclSub = poseNode.subscribe ("amcl_pose", 100, & PoseListener::amclCallback, this);
    //ROS_DEBUG ("subscribed to amcl_pose");

    // vector to store coordinates
    std::vector <double> poseAMCL;

  public:
    std::vector <double> getPose ()
    {
      return poseAMCL;
    }

    double getPoseX ()
    {
      // at (0) is the x coordinate
      return poseAMCL.at (0);
    }

    double getPoseY ()
    {
      // at (1) is the y coordinate
      return poseAMCL.at (1);
    }

    void setPose (double x, double y)
    {
      // clear the vector before pushing back
      // this will stop the vector from becoming larger than 2 doubles (x and y coordinates)
      poseAMCL.clear ();

      // insert x coordinate
      poseAMCL.push_back (x);

      // insert y coordinate
      poseAMCL.push_back (y);
    }

    void amclCallback (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & AMCLmessage)
    {
      // set vector to current pose from amcl message
      setPose (AMCLmessage -> pose.pose.position.x, AMCLmessage -> pose.pose.position.y);
    }
};

class CostmapListener
{
  private:
    ros::NodeHandle costmapNode;
    //ROS_DEBUG ("created nodehandle costmapNode");

    // subscribe to costmap to get the global costmap
    ros::Subscriber costmapSub = costmapNode.subscribe ("/move_base/global_costmap/costmap", 100, & CostmapListener::costmapCallback, this);
    //ROS_DEBUG ("subscribed to costmap");

    // vector to make reading and using costmap easier
    std::vector <int> costmap;

    void addToCostmap (int value)
    {
      costmap.push_back (value);
    }

    void costmapCallback (const nav_msgs::OccupancyGrid::ConstPtr & costmapMessage)
    {
      // clear previously stored costmap
      clearCostmap ();

      // create costmap
      for (int index = 0; index < costmapMessage -> data.size (); index += 1)
      {
        int costmapValue = costmapMessage -> data [index];

        addToCostmap (costmapValue);
      }
    }

    void clearCostmap ()
    {
      costmap.clear ();
    }

  public:
    int checkCostmap (int x, int y)
    {
      // 2d coordinates to 1d array: index = x + y * num_cols
      // https://stackoverflow.com/questions/1730961/convert-a-2d-array-index-into-a-1d-index/1730975#comment55431476_1730975
      int index = x + y * 2;

      return costmap.at (index);
    }
};

class PeopleListener
{
  private:
    ros::NodeHandle peopleNode;
    //ROS_DEBUG ("created nodehandle peopleNode");

    // subscribe to people to get locations of people from leg detection
    ros::Subscriber peopleSub = peopleNode.subscribe ("people", 100, & PeopleListener::peopleCallback, this);
    //ROS_DEBUG ("subscribed to people");

    // vector to store coordinates of people
    std::vector <std::vector <double>> peopleLocations;

    CostmapListener costmapListener;

    // filter out potential false positives (doesnt work)
    void filterPeople (std::vector <std::vector <double>> people)
    {
      for (int index = 0; index < people.size (); index += 1)
      {
        // coordinates of the current detected person
        int x = people.at (index).at (0);
        int y = people.at (index).at (1);

        // if the coordinates of the detected person are too close to an obstacle on the global costmap
        if (costmapListener.checkCostmap (x, y) >= -1)
        {
          // discard this detection
          people.erase (people.begin () + index);

          // subtract from index since the size of the vector has decreased
          index -= 1;
        }
      }
    }

  public:
    std::vector <std::vector <double>> getPeopleLocations ()
    {
      return sortByReliability ();
    }

    std::vector <std::vector <double>> sortByReliability ()
    {
      // if peopleLocations has items
      if (peopleLocations.size () > 0)
      {
        //ROS_INFO ("people present, sorting...");

        // copy the vector of people locations
        std::vector <std::vector <double>> reliabilitySorted = peopleLocations;

        // sort the copied vector
        // https://en.cppreference.com/w/cpp/algorithm/sort
        std::sort
        (
          reliabilitySorted.begin (),
          reliabilitySorted.end (),
          [] (const std::vector <double> & a, const std::vector <double> & b)
          {
            return a [2] < b [2];
          }
        );

        return reliabilitySorted;
      }

      // if peopleLocations is empty, return a vector of zeroes
      return std::vector <std::vector <double>> {{0, 0, 0}};
    }

    void setPersonLocation (double x, double y, double r)
    {
      // add to peopleLocations in push_back in format (x coordinate, y coordinate, reliability)
      peopleLocations.push_back (std::vector <double> ({x, y, r}));
    }

    void peopleCallback (const people_msgs::People::ConstPtr & peopleMessage)
    {
      // clear previously stored people locations (since people move)
      clearLocations ();

      // add in new locations of people
      for (int index = 0; index < peopleMessage -> people.size (); index += 1)
      {
        double x = peopleMessage -> people [index].position.x;
        double y = peopleMessage -> people [index].position.y;
        double r = peopleMessage -> people [index].reliability;

        setPersonLocation (x, y, r);
      }
    }

    void clearLocations ()
    {
      peopleLocations.clear ();
    }

};

#endif
