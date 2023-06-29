#include "Gesture.h"

int main (int argc, char ** argv)
{
	ros::init (argc, argv, "joint_controller");

	ros::NodeHandle jointControllerNode;

	while (jointControllerNode.ok ())
	{
		Wave waveLeft;
		Wave waveRight (false);

		waveLeft.perform ();
		waveRight.perform ();
	}

	return 0;
}
