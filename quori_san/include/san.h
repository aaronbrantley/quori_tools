#ifndef SAN_H_
#define SAN_H_

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

	point findBehaviorGoal (char goalType)
	{
		/*
		* 	search for a goal depending on
		* 	behavior
		*/
		switch (goalType)
		{
			case 'e':
			{
				ROS_INFO ("Engaging behavior...");
				Engaging behavior;

				return behavior.findGoal ();
			}
			case 'c':
			{
				ROS_INFO ("Conservative behavior...");
				Conservative behavior;

				return behavior.findGoal ();
			}
			case 'r':
			{
				ROS_INFO ("Reserved behavior...");
				Reserved behavior;

				return behavior.findGoal ();
			}
			case 's':
			{
				ROS_INFO ("Stationary behavior...");
				Stationary behavior;

				return behavior.findGoal ();
			}
			default:
			{
				ROS_INFO ("Engaging behavior (default)...");
				Engaging behavior;

				return behavior.findGoal ();
			}
		}
	}
}

#endif
