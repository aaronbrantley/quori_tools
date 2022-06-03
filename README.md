# quoriTools
programs that will make working with quori easier

## face_image
programs here take jpg files from `images/` and display them on quori's face projector. this can be accomplished by creating an `ImagePublisher` object and calling its `publishImage (std::string imageName)` function. 

### faceTest
displays a neutral expression from `images/neutral_face.jpg` on quori's face.

### expressions
displays an image depending on the input expression. right now, `images/` contains faces from twitter's emojis.
