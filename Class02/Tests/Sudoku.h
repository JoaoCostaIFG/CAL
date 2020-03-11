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
   * numbers[i][j] - número que ocupa a linha i, coluna j (de 0 a 8)
   * 0 quer dizer não preenchido.
   */
  int numbers[9][9];
  int num_sol = 0;

  /**
   * Informação derivada da anterior, para acelerar processamento (número de 1 a
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
   * Inicia um Sudoku com um conteúdo inicial.
   * Lança excepção IllegalArgumentException se os valores
   * estiverem fora da gama de 1 a 9 ou se existirem números repetidos
   * por linha, coluna ou bloc 3x3.
   *
   * @param nums matriz com os valores iniciais (0 significa por preencher)
   */
  Sudoku(int nums[9][9]);

  /**
   * Obtem o conteúdo actual (só para leitura!).
   */
  int **getNumbers();

  /**
   * Verifica se o Sudoku já está completamente resolvido
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
   * Retorna indicação de sucesso ou insucesso (sudoku impossível).
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
