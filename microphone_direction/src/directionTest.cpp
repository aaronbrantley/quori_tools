#include "DirectionListener.h"

int main ()
{
  //classes
  DirectionListener microphoneDirection;

  // use the DirectionListener class to get the direction of the last sound heard
  microphoneDirection.getSoundDirection ();

  return 0;
}
