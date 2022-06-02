#include <ros/package.h>
#include "ImagePublisher.h"

int main (int argc, char ** argv)
{
  // classes
  ImagePublisher projectorPublisher;

  // std
  std::string packagePath;
  std::string expression;
  std::string imagePath;

  // set desired expression
  expression = "smile";

  // get the path to image given desired expression
  packagePath = ros::package::getPath ("face_image");
  imagePath = packagePath + "/images/" + expression + ".jpg";

  std::cout << "publishing " << imagePath << std::endl;

  // use the publisher class to publish the image
  projectorPublisher.publishImage (argc, argv, imagePath);

  // shutdown the node since the connection is latched
  //ros::shutdown ();
}
