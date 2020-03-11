/*
 * NearestPoints.cpp
 */

#include "NearestPoints.h"
#include "Point.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <thread>

const double MAX_DOUBLE = std::numeric_limits<double>::max();

Result::Result(double dmin, Point p1, Point p2) {
  this->dmin = dmin;
  this->p1 = p1;
  this->p2 = p2;
}

Result::Result() {
  this->dmin = MAX_DOUBLE;
  this->p1 = Point(0, 0);
  this->p2 = Point(0, 0);
}

/**
 * Auxiliary functions to sort vector of points by X or Y axis.
 */
static void sortByX(vector<Point> &v, int left, int right) {
  std::sort(v.begin() + left, v.begin() + right + 1, [](Point p, Point q) {
    return p.x < q.x || (p.x == q.x && p.y < q.y);
  });
}

static void sortByY(vector<Point> &v, int left, int right) {
  std::sort(v.begin() + left, v.begin() + right + 1, [](Point p, Point q) {
    return p.y < q.y || (p.y == q.y && p.x < q.x);
  });
}

/**
 * Brute force algorithm O(N^2).
 */
Result nearestPoints_BF(vector<Point> &vp) {
  Result res;
  double curr;

  for (size_t i = 0; i < vp.size() - 1; ++i) {
    for (size_t j = i + 1; j < vp.size(); ++j) {
      curr = vp[i].distSquare(vp[j]);
      if (curr < res.dmin) {
        res.dmin = curr;
        res.p1 = vp[i];
        res.p2 = vp[j];
      }
    }
  }

  res.dmin = sqrt(res.dmin);
  return res;
}

/**
 * Improved brute force algorithm, that first sorts points by X axis.
 */
Result nearestPoints_BF_SortByX(vector<Point> &vp) {
  Result res;
  sortByX(vp, 0, vp.size() - 1);

  double curr;
  for (size_t i = 0; i < vp.size() - 1; ++i) {
    for (size_t j = i + 1; j < vp.size(); ++j) {
      if (vp[j].x - vp[i].x > res.dmin)
        break;

      curr = vp[i].distSquare(vp[j]);
      if (curr < res.dmin) {
        res.dmin = curr;
        res.p1 = vp[i];
        res.p2 = vp[j];
      }
    }
  }

  res.dmin = sqrt(res.dmin);
  return res;
}

/**
 * Auxiliary function to find nearest points when the number of cases is small
 * enough.
 */
static void npByX(vector<Point> &vp, int left, int right, Result &res) {
  double curr;
  for (int i = left; i < right - 1; ++i) {
    for (int j = i + 1; j < right; ++j) {
      if (vp.at(j).x - vp.at(i).x > res.dmin)
        break;

      if ((curr = vp.at(i).distance(vp.at(j))) < res.dmin) {
        res.dmin = curr;
        res.p1 = vp.at(i);
        res.p2 = vp.at(j);
      }
    }
  }
}

/**
 * Auxiliary function to find nearest points in strip, as indicate
 * in the assignment, with points sorted by Y coordinate.
 * The strip is the part of vp between indices left and right (inclusive).
 * "res" contains initially the best solution found so far.
 */
static void npByY(vector<Point> &vp, int left, int right, Result &res) {
  double curr;
  for (int i = left; i < right - 1; ++i) {
    for (int j = i + 1; j < right; ++j) {
      if (vp.at(j).y - vp.at(i).y > res.dmin)
        break;

      if ((curr = vp.at(i).distance(vp.at(j))) < res.dmin) {
        res.dmin = curr;
        res.p1 = vp.at(i);
        res.p2 = vp.at(j);
      }
    }
  }
}

/**
 * Recursive divide and conquer algorithm.
 * Finds the nearest points in "vp" between indices left and right (inclusive),
 * using at most numThreads.
 */
static Result np_DC(vector<Point> &vp, int left, int right, int numThreads) {
  // already sorted by X

  Result res;
  if (right - left == 1) { // Base case of two points
    res.dmin = vp.at(left).distance(vp.at(right));
    res.p1 = vp.at(left);
    res.p2 = vp.at(right);
    return res;
  } else if (right - left == 0) { // Base case of single point: MAX_DOUBLE
    res.dmin = MAX_DOUBLE;
    res.p1 = vp.at(left);
    res.p2 = vp.at(right);
    return res;
  }

  // Divide in halves (left and right) and solve them recursively
  int mid = (left + right) / 2;
  Result res_l, res_r;
  if (numThreads <= 1) {
    res_l = np_DC(vp, left, mid, numThreads);
    res_r = np_DC(vp, mid + 1, right, numThreads);
  } else {
    std::thread t([&vp, &res_l, left, mid, numThreads] {
      vector<Point> vv(vp); // copy array -> multiple threads might sort part of
                            // it at the same time
      res_l = np_DC(vv, left, mid, numThreads / 2);
    });
    res_r = np_DC(vp, mid + 1, right, numThreads / 2);
    t.join(); // wait for the other thread
  }

  // Select the best solution from left and right
  res = (res_l.dmin < res_r.dmin) ? res_l : res_r;

  // Determine the strip area around middle point
  // double mid_line = (vp.at(right).x - vp.at(left).x) / 2.0;
  // for (strip_l = left; strip_l < mid; ++strip_l) {
  // if (vp.at(strip_l).x < mid_line && vp.at(strip_l).x > mid_line - res.dmin)
  // break;
  // }
  // for (strip_r = right; strip_r > mid + 1; --strip_r) {
  // if (vp.at(strip_r).x > mid_line && vp.at(strip_r).x < mid_line + res.dmin)
  // break;
  // }
  int strip_l, strip_r;
  for (strip_l = mid; strip_l >= left; --strip_l) {
    if (vp.at(mid).x - vp.at(strip_l).x > res.dmin)
      break;
  }
  ++strip_l;
  for (strip_r = mid + 1; strip_r <= right; ++strip_r) {
    if (vp.at(strip_l).x - vp.at(mid).x > res.dmin)
      break;
  }
  --strip_r;

  // Order points in strip area by Y coordinate
  sortByY(vp, strip_l, strip_r);

  // Calculate nearest points in strip area (using npByY function)
  npByY(vp, strip_l, strip_r, res);

  // Reorder points in strip area back by X coordinate
  sortByX(vp, strip_l, strip_r);

  return res;
}

/**
 * Defines the number of threads to be used.
 */
static int numThreads = 1;
void setNumThreads(int num) { numThreads = num; }

/*
 * Divide and conquer approach, single-threaded version.
 */
Result nearestPoints_DC(vector<Point> &vp) {
  sortByX(vp, 0, vp.size() - 1);
  return np_DC(vp, 0, vp.size() - 1, 1);
}

/*
 * Multi-threaded version, using the number of threads specified
 * by setNumThreads().
 */
Result nearestPoints_DC_MT(vector<Point> &vp) {
  sortByX(vp, 0, vp.size() - 1);
  return np_DC(vp, 0, vp.size() - 1, numThreads);
}
