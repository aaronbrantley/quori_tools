#ifndef SAN_H_
#define SAN_H_

#include "Listeners.h"

namespace san
{
	VoteListener votes;
	int density = 0;
	int vulnerability = 0;
	int densityMidpoint = 5;
	int vulnerabilityMidpoint = 5;

	point findBehaviorGoal ()
	{
		std::vector <point> people = votes.getVotes ();
		density = people.size ();
		// set vulnerability to context detection value (apriltag)
		vulnerability = 0;

		if (vulnerability < vulnerabilityMidpoint)
		{
			// low vulnerability;
			if (density < densityMidpoint)
			{
				// low density
				ROS_INFO ("Engaging behavior...");
				Engaging behavior;

				return behavior.findGoal (people);
			}

			// high density
			ROS_INFO ("Conservative behavior...");
			Conservative behavior;

			return behavior.findGoal (people);
		}

		// high vulnerability
		if (density < densityMidpoint)
		{
			// low density
			ROS_INFO ("Reserved behavior...");
			Reserved behavior;

			return behavior.findGoal ();
		}

		// high density
		ROS_INFO ("Stationary behavior...");
		Stationary behavior;

		return behavior.findGoal ();
	}
}

#endif
