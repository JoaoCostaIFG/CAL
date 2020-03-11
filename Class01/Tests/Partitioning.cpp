/*
 * Partioning.cpp
 */

#include "Partitioning.h"
#include <iostream>
using namespace std;

int s_recursive(int n, int k) {
  if (k == 1 || k == n)
    return 1;
  return s_recursive(n - 1, k - 1) + k * s_recursive(n - 1, k);
}

int combinations(int n, int k) {
  int mtr_size = n - k;
  int *mtr = new int[mtr_size + 1];
  for (int i = 0; i <= mtr_size; ++i)
    mtr[i] = 1;

  for (int i = 1; i <= k; ++i) {
    for (int j = 1; j <= mtr_size; ++j) {
      mtr[j] += mtr[j - 1];
    }
  }

  // print
  for (int i = 0; i <= mtr_size; ++i)
    cout << mtr[i] << endl;

  delete[] mtr;
  return 1;
}

int s_dynamic(int n, int k) {
  if (k == 1 || k == n)
    return 1;

  int mtr_size = n - k;
  int *mtr = new int[mtr_size + 1];
  for (int i = 0; i <= mtr_size; ++i) {
    mtr[i] = 1;
  }

  for (int i = 2; i <= k; ++i) {
    for (int j = 1; j <= mtr_size; ++j) {
      // mtr[j] *= k;
      mtr[j] += i * mtr[j - 1];
    }
  }

  // print
  // for (int i = 0; i <= mtr_size; ++i)
    // cout << mtr[i] << endl;

  delete[] mtr;
  return mtr[mtr_size];
}

int b_recursive(int n) {
  int res = 0;
  for (int i = 1; i <= n; ++i) {
    res += s_recursive(n, i);
  }
  return res;
}

int b_dynamic(int n) {
  int res = 1;
  int mtr_size = n;
  int *mtr = new int[mtr_size + 1];
  for (int i = 0; i <= mtr_size; ++i) {
    mtr[i] = 1;
  }

  for (int i = 2; i <= n; ++i) {
    for (int j = 1; j <= mtr_size; ++j) {
      mtr[j] += i * mtr[j - 1];
    }
    res += mtr[n - i];
  }


  delete[] mtr;
  return res;
}
