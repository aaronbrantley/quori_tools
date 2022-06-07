#include "ImagePublisher.h"

int main ()
{
  ImagePublisher projectorPublisher;

  std::string expression;

  std::cout << "input desired expression: ";

  std::getline (std::cin, expression);

  projectorPublisher.publishImage (expression);

  return 0;
}
