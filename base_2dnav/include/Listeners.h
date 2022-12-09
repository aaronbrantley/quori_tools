#ifndef LISTENERS_H_
#define LISTENERS_H_

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <people_msgs/People.h>

/*
*   generic listener node creation
*/
class Listener
{
	protected:
		// ros
		ros::Time latest;
		ros::Duration waitTime;

		// standard
		std::string nodeName;

		/*
		*   create a ros node given a name
		*/
		void initializeRos (std::string name)
		{
			int argc = 0;
			char ** argv = nullptr;

			ros::init (argc, argv, name);
		}
};

/*
*   finds the position of the robot in the map
*   from the transform between the robot frame
*   and the map frame
*/
class PoseListener : public Listener
{
	private:
		// transform
		tf::StampedTransform transformObject;

		// standard
		std::string mapFrame;
		std::string robotFrame;
		std::vector <double> position;

		// readability
		int x = 0;
		int y = 1;

	protected:
		/*
		*   use roscpp functions here
		*/
		void initialize ()
		{
			initializeRos (nodeName);

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
		*/
		std::vector <double> getPose ()
		{
			latest = ros::Time (0);
			waitTime = ros::Duration (1);

			nodeName = "pose_listener";
			mapFrame = "map";
			robotFrame = "quori/base_link";

			initialize ();

			position.clear ();

			position.push_back (transformObject.getOrigin ().x ());
			position.push_back (transformObject.getOrigin ().y ());

			ROS_DEBUG_STREAM ("pose transform return value: (" << position [x] << ", " << position [y] << ")");

			return position;
		}
};

/*
*   finds the position of people
*   detected in the map
*/
class PeopleListener : public Listener
{
	private:
		// standard
		std::vector <double> position;
		std::vector <std::vector <double>> locations;

		// readability
		int x = 0;
		int y = 1;

	protected:
		/*
		*   use roscpp functions here
		*/
		void initialize ()
		{
			initializeRos (nodeName);

			ros::NodeHandle peopleListenerNode;
			ros::Subscriber peopleSub = peopleListenerNode.subscribe ("people", 0, & PeopleListener::peopleCallback, this);
		}

		/*
		*		sort the vector of person locations
		*/
		std::vector <std::vector <double>> sortByReliability ()
		{
			// if locations has items
			if (locations.size () > 0)
			{
				// copy the vector of people locations
				std::vector <std::vector <double>> reliabilitySorted = locations;

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

			// if locations is empty, return a vector of zeroes
			return std::vector <std::vector <double>> {{0, 0, 0}};
		}

		/*
		*
		*/
		void peopleCallback (const people_msgs::People::ConstPtr & peopleMessage)
		{
			// clear previously stored people locations
			locations.clear ();

			// add in new locations of people
			for (int index = 0; index < peopleMessage -> people.size (); index += 1)
			{
				double x = peopleMessage -> people [index].position.x;
				double y = peopleMessage -> people [index].position.y;
				double r = peopleMessage -> people [index].reliability;

				locations.push_back (std::vector <double> ({x, y, r}));
			}
		}

	public:
		std::vector <std::vector <double>> getPeopleLocations ()
		{
			return sortByReliability ();
		}
};

#endif
