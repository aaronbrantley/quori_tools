#include "Gesture.h"

int main (int argc, char ** argv)
{
	ros::init (argc, argv, "joint_controller");

	ros::NodeHandle jointControllerNode;

	while (jointControllerNode.ok ())
	{
		Wave waveLeft;
		Wave waveRight (false);
		Point pointLeft;
		Point pointRight (false);
		//DirectAttention direct;

		waveLeft.perform ();
		waveRight.perform ();
		pointLeft.perform ();
		pointRight.perform ();
		//direct.perform ();
	}

	return 0;
}
