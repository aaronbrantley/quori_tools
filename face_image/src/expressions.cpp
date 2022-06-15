#include "ImagePublisher.h"

int main ()
{
  // classes
  ImagePublisher projectorPublisher;

  // std
  std::string expression;

  // primitive
  bool validInput = false;

  // while a valid expression has not been entered
  while (!validInput)
  {
    std::cout << "enter desired expression: ";

    // user input
    std::getline (std::cin, expression);

    validInput = projectorPublisher.checkExpression (expression);

    // if invalid expression was entered
    if (!validInput)
    {
      std::cout << "invalid expression entered" << std::endl << std::endl;

      projectorPublisher.printExpressionList ();

      std::cout << std::endl;
    }
  }

  projectorPublisher.publishImage (expression);

  return 0;
}
