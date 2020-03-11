/*
 * Sum.cpp
 */

#include "Sum.h"
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

string calcSum(int *sequence, int size) {
  if (!size)
    return "";

  ostringstream os;
  int *mtr = new int[size];
  memcpy(mtr, sequence, sizeof(int) * size);
  int min = sequence[0], ind, i, j;
  for (i = 0; i < size; ++i) {
    if (sequence[i] < min) {
      min = sequence[i];
      ind = i;
    }
  }
  os << min << ',' << ind << ';';

  for (i = 1; i < size; ++i) {
    for (j = 0; j < size - i; ++j) {
      mtr[j] += sequence[j + i];
      if (mtr[j] < min || j == 0) {
        min = mtr[j];
        ind = j;
      }
    }

    os << min << ',' << ind << ';';
  }

  delete[] mtr;
  return os.str();
}

#define REPEATS 1000

void testPerformanceCalcSum(int *sequence, int size) {
  ofstream out_f;
  out_f.open("times.csv", ios::out | ios::app);
  if (!out_f.is_open()) {
    std::cout << "Not open." << endl;
    exit(1);
  }

  std::chrono::_V2::system_clock::time_point start, end;
  double tot = 0;
  for (size_t i = 0; i < REPEATS; ++i) {
    // get start time
    start = std::chrono::high_resolution_clock::now();
    // func call
    calcSum(sequence, size);
    // get end time
    end = std::chrono::high_resolution_clock::now();

    tot +=
        std::chrono::duration_cast<chrono::microseconds>(end - start).count();
  }

  // write
  out_f << size << ';' << tot / REPEATS << ';';
  out_f.close();
}
