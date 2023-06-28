#include "Gestures.h"

int main (int argc, char ** argv)
{
	ros::init (argc, argv, "joint_controller");

	ros::NodeHandle jointControllerNode;

	while (jointControllerNode.ok ())
	{
		WaveRightArm waveRight;

		waveRight.perform ();
	}

	return 0;
}
