#ifndef LISTENERS_H_
#define LISTENERS_H_

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <people_msgs/PositionMeasurementArray.h>

#include "voting/Density.h"

/*
*   finds the position of the robot in the map
*   from the transform between the robot frame
*   and the map frame
*/
class PositionListener
{
	private:
		voting::Point position;
		tf::StampedTransform transform;
		ros::Time latest = ros::Time (0);
		ros::Duration waitTime = ros::Duration (1.0);

		std::string nodeName = "position_listener";
		std::string mapFrame = "map";
		std::string robotFrame = "quori/base_link";

	protected:
		/*
		*   use roscpp functions here
		*/
		void initialize ()
		{
			tf::TransformListener transformListener;

			try
			{
				transformListener.waitForTransform (mapFrame, robotFrame, latest, waitTime);
				transformListener.lookupTransform (mapFrame, robotFrame, latest, transform);
			}
			catch (tf::TransformException & exception)
			{
				ROS_ERROR_STREAM (exception.what ());
			}
		}

	public:
		PositionListener ()
		{
				initialize ();
		}

		/*
		*   store the position of the robot
		*   into a vector
		*/
		voting::Point getPosition ()
		{
			position.x = transform.getOrigin ().x ();
			position.y = transform.getOrigin ().y ();

			ROS_DEBUG_STREAM (nodeName << " position: (" << position.x << ", " << position.y << ")");

			return position;
		}
};

/*
* 	listens to the messages
* 	sent to the lidar_voting topic
*/
class DensityListener
{
	private:
		std::string nodeName = "density_listener";
		int current;
		float average;
		std::vector <voting::Point> locations;
		ros::Subscriber voteSubscriber;

	protected:
		/*
		* 	enables ros functionality
		* 	creates listener node
		* 	and subscribes to /people
		*/
		void initialize ()
		{
			int argc = 0;
			char ** argv = nullptr;

			ros::init (argc, argv, nodeName);

			ros::NodeHandle densityListener;
			voteSubscriber = densityListener.subscribe ("density", 10, & DensityListener::densityCallback, this);

			ROS_DEBUG_STREAM ("Initialized " << nodeName);
		}

		/*
		* 	clears the previous detection vector
		* 	puts detection data into the point struct
		* 	add each detection to the detections vector
		*/
		void densityCallback (const voting::Density::ConstPtr & densityMessage)
		{
			current = densityMessage -> current;
			average = densityMessage -> average;

			locations.clear ();

			for (int index = 0; index < densityMessage -> current; index += 1)
			{
				voting::Point location;

				location.x = densityMessage -> locations [index].x;
				location.y = densityMessage -> locations [index].y;

				locations.push_back (location);
			}
		}

		public:
			DensityListener ()
			{
				initialize ();
			}

			int getCurrent ()
			{
				return current;
			}

			float getAverage ()
			{
				return average;
			}

			std::vector <voting::Point> getLocations ()
			{
				return locations;
			}
};

#endif
