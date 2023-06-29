#ifndef JOINT_VALUES_H_
#define JOINT_VALUES_H_

struct joint_values
{
	std::vector <std::string> jointNames = {"l_shoulder_pitch", "l_shoulder_roll", "r_shoulder_pitch", "r_shoulder_roll", "waist_pitch"};
	// maybe get current joint values instead of resetting to 0?
	// or maybe joints will keep their position if the goal has an empty value for them?
	float leftShoulderPitch = 0.0;
	float leftShoulderRoll = 0.0;
	float rightShoulderPitch = 0.0;
	float rightShoulderRoll = 0.0;
	float waistPitch = 0.0;
	float speed = 1.0;
};

#endif
