/*
 * Change.cpp
 */

#include "Change.h"
#include <stdlib.h>
#include <string.h>

#define MAX_INT 3200000

string calcChange(int m, int numCoins, int *coinValues) {
  if (m == 0)
    return "";

  string result;
  int *vals = (int *)malloc(sizeof(int) * (m + 1));
  memset(vals, 0, (m + 1) * sizeof(int));
  int *best = (int *)malloc(sizeof(int) * (m + 1));
  memset(best, 0, (m + 1) * sizeof(int));

  int new_val;
  for (int i = 1; i <= m; ++i) {
    vals[i] = MAX_INT;
    for (int j = 0; j < numCoins; ++j) {
      if (coinValues[j] <= i) {
        new_val = vals[i - coinValues[j]] + 1;
        if (new_val < vals[i]) {
          vals[i] = new_val;
          best[i] = coinValues[j];
        }
      }
    }
  }

  if (vals[m] == MAX_INT)
    return "-";

  int val = m;
  int n = vals[m] - 1;
  int *res = (int *)malloc(sizeof(int) * (vals[m] + 1));
  for (int i = 0; i < vals[m]; ++i) {
    res[n] = best[val];
    // dcrm
    --n;
    val -= best[val];
  }

  for (int i = 0; i < vals[m]; ++i) {
    result += to_string(res[i]) + ";";
  }

  return result;
}
