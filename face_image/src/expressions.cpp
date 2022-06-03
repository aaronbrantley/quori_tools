#include "ImagePublisher.h"

int main ()
{
  ImagePublisher projectorPublisher;
  std::string expression;

  std::cout << "input desired expression: ";

  while (expression.length () == 0)
  {
    std::getline (std::cin, expression);
  }

  projectorPublisher.publishImage (expression);

  return 0;
}
