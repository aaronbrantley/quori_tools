#include "DirectionListener.h"

int main ()
{
  //classes
  DirectionListener soundDirection;

  // use the DirectionListener class to get the direction of the last sound heard
  ROS_INFO_STREAM ("sound at " << soundDirection.getSoundDirection () << " degrees");

  return 0;
}
