#include "JointController.h"

int main (int argc, char ** argv)
{
	ros::init (argc, argv, "joint_controller");

	bool incrementWaistPitch = true;
	bool incrementShoulderPitch = true;
	bool incrementShoulderRoll = true;
	float waistPitch = -0.4;
	float shoulderPitch = -1.0;
	float shoulderRoll = -2.0;
	float increment = 0.0025;
	JointController controller;
	ros::NodeHandle jointControllerNode;

	while (jointControllerNode.ok ())
	{
		controller.setLeftShoulderPitch (shoulderPitch);
		controller.setLeftShoulderRoll (shoulderRoll);
		controller.setRightShoulderPitch (shoulderPitch);
		controller.setRightShoulderRoll (shoulderRoll);
		controller.setWaistPitch (waistPitch);
		controller.createGoal ();

		if (waistPitch < -0.4)
		{
			incrementWaistPitch = true;
		}
		else if (waistPitch > 0.2)
		{
			incrementWaistPitch = false;
		}

		if (abs (shoulderPitch) > 1.0)
		{
			incrementShoulderPitch = !incrementShoulderPitch;
		}

		if (abs (shoulderRoll) > 2.0)
		{
			incrementShoulderRoll = !incrementShoulderRoll;
		}


		if (incrementWaistPitch)
		{
			waistPitch += increment;
		}
		else
		{
			waistPitch -= increment;
		}

		if (incrementShoulderPitch)
		{
			shoulderPitch += increment;
		}
		else
		{
			shoulderPitch -= increment;
		}

		if (incrementShoulderRoll)
		{
			shoulderRoll += increment;
		}
		else
		{
			shoulderRoll -= increment;
		}
	}


	return 0;
}
