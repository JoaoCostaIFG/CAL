// #include "gtest/gtest.h"
// #include "gmock/gmock.h"
#include "Tests/Sum.h"

#include <ctime>
#include <cstdlib>

#define MAX_N 500

int main(int argc, char *argv[]) {
  srand(time(NULL));

  int sequence[MAX_N];
  int max_n;

  for (int i = 10; i <= MAX_N; i += 10) {
    max_n = 10 * i;
    for (int j = 0; j < i; ++j) {
      sequence[j] = rand() % max_n + 1;
    }
    testPerformanceCalcSum(sequence, i);
  }

  return 0;

  // testing::InitGoogleTest(&argc, argv);
  // std::cout << "CAL Lab Class 01" << std::endl;
  // return RUN_ALL_TESTS();;
}
