#ifndef LISTENERS_H_
#define LISTENERS_H_

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <people_msgs/PositionMeasurementArray.h>

#include "point.h"

/*
*   finds the position of the robot in the map
*   from the transform between the robot frame
*   and the map frame
*/
class PositionListener
{
	private:
		point position;
		tf::StampedTransform transform;
		ros::Time latest;
		ros::Duration waitTime;

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
		point getPosition ()
		{
			latest = ros::Time (0);
			waitTime = ros::Duration (1);

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
class VoteListener
{
	private:
		std::string nodeName = "vote_listener";
		std::vector <point> votes;
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

			ros::NodeHandle voteListener;
			voteSubscriber = voteListener.subscribe ("people_voting", 0, & VoteListener::voteCallback, this);

			ROS_DEBUG_STREAM ("Initialized " << nodeName);
		}

		/*
		* 	clears the previous detection vector
		* 	puts detection data into the point struct
		* 	add each detection to the detections vector
		*/
		void voteCallback (const people_msgs::PositionMeasurementArray::ConstPtr & voteMessage)
		{
			votes.clear ();

			for (int index = 0; index < voteMessage -> people.size (); index += 1)
			{
				point voted;

				voted.x = voteMessage -> people [index].pos.x;
				voted.y = voteMessage -> people [index].pos.y;

				votes.push_back (voted);
			}
		}

		public:
			VoteListener ()
			{
				initialize ();
			}

			std::vector <point> getVotes ()
			{
				return votes;
			}
};

#endif
