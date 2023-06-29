#ifndef GESTURE_H_
#define GESTURE_H_

#include "JointController.h"

/*
* 	base class for every gesture
*/
class Gesture
{
	private:
		/*
		* 	the robot's current pose
		* 	can be seen as an animation's current "frame"
		*/
		int poseIndex = 0;
		// the list of poses that the robot will perform
		std::vector <joint_values> poses;
		// sends messages to move the robot's joints
		JointController controller;

	protected:
		/*
		* 	adds to the poses vector
		*/
		void addPose (joint_values newPose)
		{
			poses.push_back (newPose);
		}

		/*
		* 	sets the parameters for the robot's joints given a pose
		* 	creates a goal from this, then sends it to the action server
		*/
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
		/*
		* 	the robot will go into a pose
		* 	then continue to the next
		* 	until there are no more poses left in the poses vector
		*/
		void perform ()
		{
			for (poseIndex; poseIndex < poses.size (); poseIndex += 1)
			{
				setPoseGoal ();
			}
		}
};

/*
* 	does a waving gesture
* 	support for left or right arm
*/
class Wave : public Gesture
{
	// declare the different poses for this gesture
	private:
		joint_values raiseArm;
		joint_values waveArm;
		joint_values waveArmBack;
		joint_values lowerArm;

	public:

		/*
		* 	waves the left arm
		*/
		Wave ()
		{
			raiseArm.leftShoulderPitch = 2.3;
			raiseArm.leftShoulderRoll = -1.0;
			raiseArm.rightShoulderRoll = -1.0;
			waveArm.leftShoulderPitch = 2.3;
			waveArm.leftShoulderRoll = -0.5;
			waveArm.rightShoulderRoll = -1.0;
			waveArmBack.leftShoulderPitch = 2.3;
			waveArmBack.leftShoulderRoll = -1.0;
			waveArmBack.rightShoulderRoll = -1.0;
			lowerArm.leftShoulderPitch = 0.0;
			lowerArm.leftShoulderRoll = -1.0;
			lowerArm.rightShoulderRoll = -1.0;

			// how quickly the robot will transition to the next pose
			raiseArm.speed = 1.0;
			waveArm.speed = 0.25;
			waveArmBack.speed = 0.25;
			lowerArm.speed = 1.0;

			// add to the poses vector
			addPose (raiseArm);
			addPose (waveArm);
			addPose (waveArmBack);
			addPose (waveArm);
			addPose (waveArmBack);
			addPose (lowerArm);
		}

		/*
		* 	choose which arm to wave
		* 	true for left arm
		* 	false for right arm
		*/
		Wave (bool arm)
		{
			// left arm
			if (arm)
			{
				raiseArm.leftShoulderPitch = 2.3;
				raiseArm.leftShoulderRoll = -1.0;
				raiseArm.rightShoulderRoll = -1.0;
				waveArm.leftShoulderPitch = 2.3;
				waveArm.leftShoulderRoll = -0.5;
				waveArm.rightShoulderRoll = -1.0;
				waveArmBack.leftShoulderPitch = 2.3;
				waveArmBack.leftShoulderRoll = -1.0;
				waveArmBack.rightShoulderRoll = -1.0;
				lowerArm.leftShoulderPitch = 0.0;
				lowerArm.leftShoulderRoll = -1.0;
				lowerArm.rightShoulderRoll = -1.0;
			}
			// right arm
			else
			{
				raiseArm.rightShoulderPitch = 2.3;
				raiseArm.rightShoulderRoll = -1.0;
				raiseArm.leftShoulderRoll = -1.0;
				waveArm.rightShoulderPitch = 2.3;
				waveArm.rightShoulderRoll = -0.5;
				waveArm.leftShoulderRoll = -1.0;
				waveArmBack.rightShoulderPitch = 2.3;
				waveArmBack.rightShoulderRoll = -1.0;
				waveArmBack.leftShoulderRoll = -1.0;
				lowerArm.rightShoulderPitch = 0;
				lowerArm.rightShoulderRoll = -1.0;
				lowerArm.leftShoulderRoll = -1.0;
			}

			raiseArm.speed = 1.0;
			waveArm.speed = 0.25;
			waveArmBack.speed = 0.25;
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
