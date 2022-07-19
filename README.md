# quori_tools
programs that will make working with quori easier

## base_2dnav
handles quori's 2d autonomous navigation.

### `roslaunch base_2dnav base_2dnav.launch`
starts up everything needed to have quori use 2d navigation in a map.

### `rosrun base_2dnav quori_wander`
makes quori pick a random point on the map and travel to it.

## face_image
takes png files and displays them on quori's face projector.

### `rosrun face_image faceTest`
displays a neutral expression from the `/images/neutral_face.png` directory onto quori's face.

### `rosrun face_image expressions`
displays an image depending on the input expression. if you would like to add your own image, make sure it is a 1280x720 .png file and put it in `/images/`. For best visibility, make the image fully black and white.

## microphone_direction
takes the latest detected sound from the `/sound_direction` topic and perform tasks with the given information.

### `rosrun microphone_direction directionTest`
prints the direction of the latest detected sound. can also be used as a calibration tool for the microphone.

### `rosrun microphone_direction turnToSound`
rotate quori to face the direction of the last sound heard.
