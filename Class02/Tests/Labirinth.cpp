/*
 * labirinth.cpp
 */

#include "Labirinth.h"

#include <iostream>
using namespace std;

Labirinth::Labirinth(int values[10][10]) {
  for (int i = 0; i < 10; i++)
    for (int j = 0; j < 10; j++)
      labirinth[i][j] = values[i][j];

  this->initializeVisited();
}

void Labirinth::initializeVisited() {
  for (int i = 0; i < 10; i++)
    for (int j = 0; j < 10; j++)
      visited[i][j] = false;
}

void Labirinth::printLabirinth() {
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 10; j++)
      cout << labirinth[i][j] << " ";

    cout << endl;
  }
}

bool Labirinth::findGoal(int x, int y) {
  // check if wall
  if (labirinth[x][y] == 0)
    return false;
  // check if exit
  if (labirinth[x][y] == 2)
    return true;
  // check if already visited
  if (visited[x][y])
    return false;
  visited[x][y] = true;

  if (x - 1 >= 0 && this->findGoal(x - 1, y))
    return true;
  if (x + 1 < 10 && this->findGoal(x + 1, y))
    return true;
  if (y - 1 >= 0 && this->findGoal(x, y - 1))
    return true;
  if (y + 1 < 10 && this->findGoal(x, y + 1))
    return true;

  return false;
}
