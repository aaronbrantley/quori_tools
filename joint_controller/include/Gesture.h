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

class Point : public Gesture
{
	private:
		joint_values point;
		joint_values lower;

	public:
		/*
		* 	point with the left arm
		*/
		Point ()
		{
			point.leftShoulderPitch = 2.0;
			point.leftShoulderRoll = -1.0;
			point.rightShoulderPitch = -0.5;
			point.rightShoulderRoll = -1.0;
			lower.leftShoulderRoll = -1.0;
			lower.rightShoulderRoll = -1.0;

			addPose (point);
			addPose (lower);
		}

		/*
		* 	choose which arm to point with
		*/
		Point (bool arm)
		{
			// left
			if (arm)
			{
				point.leftShoulderPitch = 2.0;
				point.leftShoulderRoll = -1.0;
				point.rightShoulderPitch = -0.5;
				point.rightShoulderRoll = -1.0;
			}
			// right
			else
			{
				point.rightShoulderPitch = 2.0;
				point.rightShoulderRoll = -1.0;
				point.leftShoulderPitch = -0.5;
				point.leftShoulderRoll = -1.0;
			}

			lower.leftShoulderRoll = -1.0;
			lower.rightShoulderRoll = -1.0;

			addPose (point);
			addPose (lower);
		}
};

class DirectAttention : public Gesture
{
	private:
		joint_values one;
		joint_values two;
		joint_values three;
		joint_values four;

	public:
		DirectAttention ()
		{
			one.leftShoulderPitch = 1.0;
			one.leftShoulderRoll = -1.0;
			one.rightShoulderPitch = -0.1;
			one.rightShoulderRoll = 1.0;
			two.leftShoulderPitch = 2.25;
			two.leftShoulderRoll = -1.0;
			two.rightShoulderPitch = -1.0;
			two.rightShoulderRoll = 0.75;
			two.waistPitch -0.25;
			three.leftShoulderPitch = 1.5;
			three.leftShoulderRoll = -0.25;
			three.rightShoulderPitch = -0.5;
			three.rightShoulderRoll = 1.0;
			four.leftShoulderPitch = 2.25;
			four.leftShoulderRoll = 0.333;
			four.rightShoulderPitch = -1.5;
			four.rightShoulderRoll = 1.0;
			four.waistPitch = 0.05;

			two.speed = 0.5;
			three.speed = 0.5;
			four.speed = 0.25;

			addPose (one);
			addPose (two);
			addPose (three);
			addPose (four);
		}
};

#endif
