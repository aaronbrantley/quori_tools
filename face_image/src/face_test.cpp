#include "ImagePublisher.h"

int main ()
{
  // classes
  ImagePublisher projectorPublisher;

  // use the publisher class to publish the image
  projectorPublisher.publishImage ("neutral_face");

  return 0;
}
