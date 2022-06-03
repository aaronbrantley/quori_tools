#include "ImagePublisher.h"

int main (int argc, char ** argv)
{
  // classes
  ImagePublisher projectorPublisher;

  // use the publisher class to publish the image
  projectorPublisher.publishImage ("neutral");

  return 0;
}
