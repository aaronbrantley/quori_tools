# quori_tools
programs that will make working with quori easier

## base_2dnav
handles quori's 2d autonomous navigation.

### quori_wander
makes quori wander around a map.

## face_image
takes png files and displays them on quori's face projector.

### faceTest
displays a neutral expression from the `/images/neutral_face.png` directory onto quori's face.

### expressions
displays an image depending on the input expression. if you would like to add your own image, make sure it is a 1280x720 .png file and put it in `/images/`. For best visibility, make the image fully black and white.

## microphone_direction
takes the latest detected sound from the `/sound_direction` topic and perform tasks with the given information.

### directionTest
prints the direction of the latest detected sound. can also be used as a calibration tool for the microphone.

### turnToSound
rotate quori to face the direction of the last sound heard.
