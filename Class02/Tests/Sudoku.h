/*
 * Sudoku.h
 *
 */

#ifndef SUDOKU_H_
#define SUDOKU_H_

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <time.h>
using namespace std;

#define IllegalArgumentException -1

class Sudoku {
  /**
   * numbers[i][j] - n�mero que ocupa a linha i, coluna j (de 0 a 8)
   * 0 quer dizer n�o preenchido.
   */
  int numbers[9][9];
  int num_sol = 0;

  /**
   * Informa��o derivada da anterior, para acelerar processamento (n�mero de 1 a
   * 9, nao usa 0)
   */
  int countFilled;
  bool colHasNumber[9][10];
  bool rowHasNumber[9][10];
  bool boxHasNumber[3][3][10];

  void initialize();

public:
  /** Inicia um Sudoku vazio.
   */
  Sudoku();

  /**
   * Inicia um Sudoku com um conte�do inicial.
   * Lan�a excep��o IllegalArgumentException se os valores
   * estiverem fora da gama de 1 a 9 ou se existirem n�meros repetidos
   * por linha, coluna ou bloc 3x3.
   *
   * @param nums matriz com os valores iniciais (0 significa por preencher)
   */
  Sudoku(int nums[9][9]);

  /**
   * Obtem o conte�do actual (s� para leitura!).
   */
  int **getNumbers();

  /**
   * Verifica se o Sudoku j� est� completamente resolvido
   */
  bool isComplete();

  void setNum(int row, int col, int num);
  void unsetNum(int row, int col, int num);
  bool isValidNum(int row, int col, int num);

  /**
   * Get next best cell to work on (greedy approach).
   * @note  No need to make a special case for any cnt == 0, because that would
   *        mean the sudoku is already solved.
   * @warning Only call this function if there are empty cells (check if sudoku
   *          is solved).
   */
  void getNextCell(int *row, int *col);

  /**
   * Resolve o Sudoku.
   * Retorna indica��o de sucesso ou insucesso (sudoku imposs�vel).
   */
  bool solve();
  int getNumSols() { return this->num_sol; }
  bool numsolve(int curr_branch = 0);

  /**
   * Imprime o Sudoku.
   */
  void print();
};

#endif /* SUDOKU_H_ */
