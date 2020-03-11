/*
 * Sudoku.cpp
 *
 */

#include "Sudoku.h"

/** Inicia um Sudoku vazio.
 */
Sudoku::Sudoku() { this->initialize(); }

/**
 * Inicia um Sudoku com um conteúdo inicial.
 * Lança excepção IllegalArgumentException se os valores
 * estiverem fora da gama de 1 a 9 ou se existirem números repetidos
 * por linha, coluna ou bloc 3x3.
 *
 * @param nums matriz com os valores iniciais (0 significa por preencher)
 */
Sudoku::Sudoku(int nums[9][9]) {
  this->initialize();

  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++) {
      if (nums[i][j] != 0) {
        int n = nums[i][j];
        numbers[i][j] = n;
        rowHasNumber[i][n] = true;
        colHasNumber[j][n] = true;
        boxHasNumber[i / 3][j / 3][n] = true;
        ++countFilled;
      }
    }
  }
}

void Sudoku::initialize() {
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 9; j++) {
      for (int n = 0; n < 10; n++) {
        numbers[i][j] = 0;
        rowHasNumber[i][n] = false;
        colHasNumber[j][n] = false;
        boxHasNumber[i / 3][j / 3][n] = false;
      }
    }
  }

  this->countFilled = 0;
}

/**
 * Obtem o conteúdo actual (só para leitura!).
 */
int **Sudoku::getNumbers() {
  int **ret = new int *[9];

  for (int i = 0; i < 9; i++) {
    ret[i] = new int[9];

    for (int a = 0; a < 9; a++)
      ret[i][a] = numbers[i][a];
  }

  return ret;
}

/**
 * Verifica se o Sudoku já está completamente resolvido
 */
bool Sudoku::isComplete() { return countFilled == 9 * 9; }

bool Sudoku::isValidNum(int row, int col, int num) {
  return (!rowHasNumber[row][num] && !colHasNumber[col][num] &&
          !boxHasNumber[row / 3][col / 3][num]);
}

/**
 * Get next best cell to work on (greedy approach).
 * @note  No need to make a special case for any cnt == 0, because that would
 *        mean the sudoku is already solved.
 * @warning Only call this function if there are empty cells (check if sudoku
 *          is solved).
 */
void Sudoku::getNextCell(int *row, int *col) {
  /* sudoku board size is small and fixed which makes transversing all cells
   * cheaper than storing info for later */
  int n_row, n_col, i, j;
  int min = 10, cnt;

  // check row with less zeros
  for (i = 0; i < 9; ++i) {
    cnt = 0;
    n_row = i;
    for (j = 0; j < 9; ++j) {
      if (numbers[i][j] == 0) {
        ++cnt;
        n_col = j;
      }
    }
    if (cnt && cnt < min) {
      *row = n_row;
      *col = n_col;
      min = cnt;
      if (min == 1) // if best case reached, return
        return;
    }
  }
  if (min == 0) // only needs to be checked once (no free spots)
    return;

  // check col with less zeros
  for (j = 0; j < 9; ++j) {
    cnt = 0;
    n_col = j;
    for (i = 0; i < 9; ++i) {
      if (numbers[i][j] == 0) {
        ++cnt;
        n_row = i;
      }
    }
    if (cnt && cnt < min) {
      min = cnt;
      *row = n_row;
      *col = n_col;
      if (min == 1) // if best case reached, return
        return;
    }
  }

  // check box with less zeros
  // int bi = 0, bj = 0;
  // for (int n = 0; n < 9; ++n) {
    // cnt = 0;
    // for (i = bi; i < bi + 3; ++i) {
      // for (j = bj; j < bj + 3; ++j) {
        // if (numbers[i][j] == 0) {
          // ++cnt;
          // n_row = i;
          // n_col = j;
        // }
      // }
    // }
    // if (cnt && cnt < min) {
      // *row = n_row;
      // *col = n_col;
      // min = cnt;
      // if (min == 1) // if best case reached, return
        // return;
    // }

    // if (bj == 6) {
      // ++bi;
      // bj = 0;
    // } else {
      // bj += 3;
    // }
  // }
}

void Sudoku::setNum(int row, int col, int num) {
  ++countFilled;
  this->numbers[row][col] = num;

  this->colHasNumber[col][num] = true;
  this->rowHasNumber[row][num] = true;
  this->boxHasNumber[row / 3][col / 3][num] = true;
}

void Sudoku::unsetNum(int row, int col, int num) {
  --countFilled;
  this->numbers[row][col] = 0;

  this->colHasNumber[col][num] = false;
  this->rowHasNumber[row][num] = false;
  this->boxHasNumber[row / 3][col / 3][num] = false;
}

/**
 * Resolve o Sudoku.
 * Retorna indicação de sucesso ou insucesso (sudoku impossível).
 */
bool Sudoku::solve() {
  if (this->isComplete()) // quickly check if we're already done
    return true;

  /* get next cell to work on */
  int col, row;
  this->getNextCell(&row, &col);

  /* branch for each 'valid' number on that cell */
  for (int i = 1; i <= 9; ++i) {
    if (this->isValidNum(row, col, i)) {
      setNum(row, col, i);
      if (this->solve())
        return true;
      unsetNum(row, col, i);
    }
  }
  /* report failure if this is reached */
  return false;
}

bool Sudoku::numsolve(int curr_branch) {
  if (this->isComplete()) {// quickly check if we're already done
    ++this->num_sol;
    return true;
  } 

  bool found_sol = false;
  /* get next cell to work on */
  int col, row;
  this->getNextCell(&row, &col);

  /* branch for each 'valid' number on that cell */
  for (int i = 1; i <= 9; ++i) {
    if (this->isValidNum(row, col, i)) {
      setNum(row, col, i);
      if (this->numsolve(curr_branch + 1)) {
        found_sol = true;
      }
      unsetNum(row, col, i);
    }
  }
  /* report failure if this is reached */
  return found_sol;
}

/**
 * Imprime o Sudoku.
 */
void Sudoku::print() {
  for (int i = 0; i < 9; i++) {
    for (int a = 0; a < 9; a++)
      cout << this->numbers[i][a] << " ";

    cout << endl;
  }
}
