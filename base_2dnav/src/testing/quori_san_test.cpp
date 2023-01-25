#include "Behaviors.h"
#include "san.h"

/*
* 	return a letter given its position
* 	in the alphabet
*/
char getRandomBehavior ()
{
	// all valid behaviors
	std::string behaviors = "ecrs";

	// random number generation
	std::srand (std::time (nullptr));

	int randomNumber = std::rand () % behaviors.size ();

	return behaviors.at (randomNumber);
}

int main (int argc, char ** argv)
{
	// classes
	Behaviors behavior;

	// initialize node
	ros::init (argc, argv, "quori_san");
	ros::NodeHandle sanNode;
	ros::Rate loopRate (1);

	/*
	* 	spinOnce has to be called twice
	* 	before getting correct pose,
	* 	there's probably a better way to do
	* 	this
	*/
	ros::spinOnce ();
	loopRate.sleep ();
	ros::spinOnce ();

	/*
	* 	loop until ctrl-c is pressed or
	* 	ros::shutdown is called
	*/
	while (ros::ok ())
	{
		/*
		* 	room density
		* 	0 = empty
		* 	10 = crowded
		*/
		double density = 0;

		/*
		* 	room vulnerability
		* 	0 = sociable
		* 	10 = serious
		*/
		double vulnerability = 0;

		std::vector <double> goal;
		bool goalReached = false;

		/*
		* 	set a goal based on random
		* 	selection of a behavior
		*/
		goal = san::findBehaviorGoal (getRandomBehavior ());
		goalReached = behavior.goToGoal (goal);

		if (goalReached)
		{
			ROS_INFO ("behavior finished successfully\n");
		}

		else
		{
			ROS_WARN ("behavior not successful\n");
		}

		ros::spinOnce ();
		loopRate.sleep();
	}

	return 0;
}
