#ifndef GESTURES_H_
#define GESTURES_H_

#include "JointController.h"

class Gesture
{
	private:
		int poseIndex = 0;
		std::vector <joint_values> poses;
		JointController controller;

	protected:
		void addPose (joint_values newPose)
		{
			poses.push_back (newPose);
		}

		void setPoseGoal ()
		{
			controller.setLeftShoulderPitch (poses [poseIndex].leftShoulderPitch);
			controller.setLeftShoulderRoll (poses [poseIndex].leftShoulderRoll);
			controller.setRightShoulderPitch (poses [poseIndex].rightShoulderPitch);
			controller.setRightShoulderRoll (poses [poseIndex].rightShoulderRoll);
			controller.setWaistPitch (poses [poseIndex].waistPitch);

			controller.createGoal (poses [poseIndex].speed);
		}

	public:
		void perform ()
		{
			for (poseIndex; poseIndex < poses.size (); poseIndex += 1)
			{
				setPoseGoal ();
			}
		}
};

class WaveRightArm : public Gesture
{
private:
	joint_values raiseArm;
	joint_values waveArm;
	joint_values waveArmBack;
	joint_values lowerArm;

public:
	WaveRightArm ()
	{
		raiseArm.rightShoulderPitch = -0.25;
		raiseArm.rightShoulderRoll = 1.0;
		raiseArm.speed = 1.0;

		waveArm.rightShoulderPitch = -0.25;
		waveArm.rightShoulderRoll = 0.75;
		waveArm.speed = 0.25;

		waveArmBack.rightShoulderPitch = -0.25;
		waveArmBack.rightShoulderRoll = 1.0;
		waveArmBack.speed = 0.25;

		lowerArm.rightShoulderPitch = -2.3;
		lowerArm.rightShoulderRoll = 1.0;
		lowerArm.speed = 1.0;

		addPose (raiseArm);
		addPose (waveArm);
		addPose (waveArmBack);
		addPose (waveArm);
		addPose (waveArmBack);
		addPose (lowerArm);
	}
};

#endif
