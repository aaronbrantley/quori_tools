## joint_controller
Sends goals to Quori's joint controller. Has support for gesture creation, which allows Quori to perform animations

### `rosrun joint_controller controller_test`
Has each of Quori's controllable joints go through most of its possible values

### `rosrun joint_controller gesture_test`
Plays different gestures that have been created for Quori to perform

You can create your own gestures for Quori to execute:
<ol>
  <li> Start Quori's joint controllers: `roslaunch quori_controller quori_control_holo.launch` </li>
  <li> Open the joint trajectory controller: `rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller` </li>
  <li> Use the controller to set a pose for Quori. Keep note of the joint values </li>
  <li> In `include/Gesture.h`, create a new class that uses `Gesture` as a base class </li>
  <li> For each pose that you want to include in the gesture, create a `joint_values` object </li>
  <li> `joint_values` objects have `leftShoulderPitch`, `leftShoulderRoll`, `rightShoulderPitch`, `rightShoulderRoll`, and `waistPitch` values. Copy the values from the joint controller to the joint_values objects in your class </li>
  <li> You can also configure the time it takes to go from one pose to another. `joint_values` objects also have a `speed` value, which denotes the amount of time it takes to reach the pose (in seconds) </li>
  <li> When you are done creating and configuring your poses, you can add them to the gesture with `addPose (<pose name>)` </li>
</ol>
Example gestures are available in `include/Gesture.h` 
