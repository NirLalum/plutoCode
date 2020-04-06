// defining a generic matrix

#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include <exception>
#include <string>
#include <iostream>

using namespace std;

// matrix exeptions class definition
class MatrixExeption : public exception {
public:
	MatrixExeption(string Error) : Error_(Error) {}
	const char* what() const noexcept {
		return Error_.c_str();
	}
protected:
	string Error_;
};

template <class T>
class Matrix {
public:
	Matrix(int rows, int cols) : cols_(cols), rows_(rows) {
		int i;
		T* arr;
		arr = new T[rows * cols];
		// initializing matrix
		for (i = 0; i < rows * cols; i++) {
			arr[i] = 0;
		}
		matrix_ = arr;
	}

	// example of a vector for a 2by2 matrix: [Mat[0,0] Mat[0,1], Mat[1,0], Mat[1,1]]
	void MatrixFromVector(vector<T>& initializeVec) {
		if (initializeVec.size() != rows_ * cols_) throw MatrixExeption("vector should be able to fill the whole matrix");
		int i = 0, j = 0;
		typename vector<T>::iterator it = initializeVec.begin();
		for (i=1; i <= rows_; i++) {
			for (j=1; j <= cols_; j++) {
				setElement(i, j, *it);
				it++;
			}
		}
	}

	int getRowsNum() const {
		return rows_;
	}
	int getColumnsNum() const {
		return cols_;
	}
	
	T& getElement(int row, int col) const {
		row--; // so we can send row and col num starting from 1 and not from 0 
		col--;
		return matrix_[row * cols_ + col];
	}

	void setElement(int row, int col, T elem) {
		row--;
		col--;
		matrix_[row * cols_ + col] = elem;
	}

	//overlapping operator =
	Matrix& operator=(Matrix& srcMat) {
		if (rows_ != srcMat.getRowsNum() || cols_ != srcMat.getColumnsNum()) throw MatrixExeption("both matrixes should have the same number of rows and cols");
		int i = 1, j = 1;
		for (i = 1; i <= rows_; i++) {
			for (j = 1; j <= cols_; j++) {
				setElement(i, j, srcMat.getElement(i, j));
			}
		}
	}

	void printMatrix() {
		int i = 1, j = 1;
		for (i = 1; i <= rows_; i++) {
			for (j = 1; j <= cols_; j++) {
				cout << getElement(i, j) << " ";
			}
			cout << endl;
		}
	}

	// get the transpose of "this" matrix
	Matrix& getTranspose() {
		Matrix<T> solMat(getColumnsNum(), getRowsNum());
		int i = 0, j = 0;
		T elem;
		for (i; i < rows_; i++) {
			for (j; j < cols_; j++) {
				elem = getElement(i, j);
				solMat.setElement(j, i, elem);
			}
		}
		return solMat;
	}

	// this function multiply "this" matrix with another one
	Matrix& operator*(Matrix& MatB) {
		if (cols_ != MatB.getRowsNum()) throw MatrixExeption("first matrix rows not equal to second matrix cols");
		Matrix<T> solMat(rows_, MatB.getColumnsNum());
		int i = 1, j = 1, currRow = 1;
		T solElem, MatAelem, MatBelem;
		for (i=1; i <= rows_; i++) {
			for (j=1; j <= MatB.getColumnsNum(); j++) {
				for (currRow=1; currRow <= MatB.getRowsNum(); currRow++) {
					MatAelem = getElement(i, currRow);
					MatBelem = MatB.getElement(currRow, j);
					solElem += MatAelem * MatBelem;
				}
				solMat.setElement(i, j, solElem);
				solElem = 0;
			}
		}
		return solMat;
	}

	// adds 2 matrixes
	Matrix& operator+(Matrix& MatB) {
		if (cols_ != MatB.getColumnsNum() || rows_ != MatB.getRowsNum()) throw MatrixExeption("both matrixes should have the same number of rows and cols");
		Matrix<T> solMat(rows_, cols_);
		int i = 1, j = 1;
		T solElem, MatAelem, MatBelem;
		for (i=1; i <= rows_; i++) {
			for (j=1; j <= cols_; j++) {
				MatAelem = getElement(i, j);
				MatBelem = MatB.getElement(i, j);
				solElem = MatAelem + MatBelem;
				solMat.setElement(i, j, solElem);
			}
		}
		return solMat;
	}

	// substract 2 matrixes
	Matrix& operator-(Matrix& MatB) {
		if (cols_ != MatB.getColumnsNum() || rows_ != MatB.getRowsNum()) throw MatrixExeption("both matrixes should have the same number of rows and cols");
		Matrix<T> solMat(rows_, cols_);
		int i = 1, j = 1;
		T solElem, MatAelem, MatBelem;
		for (i=1; i <= rows_; i++) {
			for (j=1; j <= cols_; j++) {
				MatAelem = getElement(i, j);
				MatBelem = MatB.getElement(i, j);
				solElem = MatAelem - MatBelem;
				solMat.setElement(i, j, solElem);
			}
		}
		return solMat;
	}

	~Matrix() {
		delete[]matrix_;
	}

protected:
	int rows_;
	int cols_;
	T* matrix_;
};

#endif

