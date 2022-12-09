#ifndef SAN_H_
#define SAN_H_

#include "Listeners.h"

namespace san
{
	/*
	* 	pick a behavior depending on the
	* 	room's density and vulnerability
	*/
	char readRoom (double density, double vulnerability)
	{
		int mediumDensity = 5;
		int mediumVulnerability = 5;

		if (vulnerability < mediumVulnerability)
		{
			// low vulnerability;
			if (density < mediumDensity)
			{
				// low density
				// engaging behavior
				return 'e';
			}

			// high density
			// conservative behavior
			return 'c';
		}

		// high vulnerability
		if (density < mediumDensity)
		{
			// low density
			// reserved behavior
			return 'r';
		}

		// high density
		// stationary behavior
		return 's';
	}

	std::vector <double> findBehaviorGoal (char goalType)
	{
		// classes
		PoseListener currentPose;
		PeopleListener personPositions;

		std::vector <double> goal;
		std::vector <double> robotLocation = currentPose.getPose ();
		std::vector <std::vector <double>> peopleLocations = personPositions.getPeopleLocations ();

		/*
		* 	search for a goal depending on
		* 	behavior
		*/
		switch (goalType)
		{
			// engaging
			case 'e':
			{
				ROS_INFO ("engaging behavior");

				Engaging engagingBehavior;
				goal = engagingBehavior.findGoal (robotLocation, peopleLocations);

				break;
			}

			// conservative
			case 'c':
			{
				ROS_INFO ("conservative behavior");

				Conservative conservativeBehavior;
				goal = conservativeBehavior.findGoal (robotLocation, peopleLocations);

				break;
			}

			// reserved
			case 'r':
			{
				ROS_INFO ("reserved behavior");

				Reserved reservedBehavior;
				goal = reservedBehavior.findGoal (robotLocation, peopleLocations);

				break;
			}

			// stationary
			case 's':
			{
				ROS_INFO ("stationary behavior");

				Stationary stationaryBehavior;
				goal = stationaryBehavior.findGoal (robotLocation, peopleLocations);

				break;
			}
		}

		return goal;
	}
}

#endif
