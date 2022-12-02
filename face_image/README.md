## face_image
takes png files and displays them on quori's face projector.

### `rosrun face_image faceTest`
displays a neutral expression from the `/images/neutral_face.png` directory onto quori's face.

### `rosrun face_image expressions`
displays an image depending on the input expression. if you would like to add your own image, make sure it is a 1280x720 .png file and put it in `/images/`. For best visibility and results, make the image fully black and white.
