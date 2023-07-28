#include "ImagePublisher.h"

int main (int argc, char ** argv)
{
	ros::init (argc, argv, "eyes_direction_test");

	ros::NodeHandle eyesDirectionTest;
	ros::Rate loopRate (1);
	std::vector <std::string> directions = {"eyes_forward", "eyes_left", "eyes_right", "eyes_up", "eyes_down"};
	int iteration = 0;

	while (eyesDirectionTest.ok ())
	{
		ImagePublisher eyePublisher;
		
		eyePublisher.publishImage (directions [iteration]);
		ROS_INFO_STREAM ("Publishing " << directions [iteration]);
		iteration += 1;

		if (iteration == directions.size ())
		{
			iteration = 0;
		}

		ros::spinOnce ();
		loopRate.sleep ();
	}

	return 0;
}
