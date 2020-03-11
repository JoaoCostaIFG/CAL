/*
 * Factorial.cpp
 */

#include "Factorial.h"

int factorialRecurs(int n)
{
  if (n == 0 || n == 1)
    return 1;
	return n * factorialRecurs(n - 1);
}

int factorialDinam(int n)
{
  long result = 1;
  while (n) {
    result *= n;
    --n;
  }

	return result;
}
