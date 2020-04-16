// defining a generic matrix

#ifndef GENERICMATRIX_H
#define GENERICMATRIX_H

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
	Matrix(int rows = 4, int cols = 4) : cols_(cols), rows_(rows) {
		int i;
		T* arr;
		arr = new T[rows * cols];
		// initializing matrix
		for (i = 0; i < rows * cols; i++) {
			arr[i] = 0;
		}
		matrix_ = arr;
	}

	Matrix(int rows, int cols, vector<T>& initializeVec) : cols_(cols), rows_(rows) {
		int i;
		T* arr;
		arr = new T[rows * cols];
		// initializing matrix
		for (i = 0; i < rows * cols; i++) {
			arr[i] = 0;
		}
		matrix_ = arr;
		MatrixFromVector(initializeVec);
	}

	// copy ctor
	Matrix(const Matrix<T>& srcMatrix) : Matrix(srcMatrix.getRowsNum(), srcMatrix.getColumnsNum()) {
		int i = 1, j = 1;
		for (i = 1; i <= srcMatrix.getRowsNum(); i++) {
			for (j = 1; j <= srcMatrix.getColumnsNum(); j++) {
				setElement(i, j, srcMatrix.getElement(i, j));
			}
		}
	}

	// getiing a part of the matrix
	Matrix<T> sliceMatrix(int rowInit, int rowFin, int colInit, int colFin) {
		if (rowFin - rowInit + 1 > rows_ || colFin - colInit + 1 > cols_) throw MatrixExeption("error in slice matrix: the sliced matrix is not contained in original Matrix");
		Matrix<T> solMat(rowFin - rowInit + 1, colFin - colInit + 1);
		int i = rowInit, j = colInit, solRowIndex = 1, solColIndex = 1;
		for (i = rowInit, solRowIndex = 1; i <= rowFin; i++, solRowIndex++) {
			for (j = colInit, solColIndex = 1; j <= colFin; j++, solColIndex++) {
				solMat.setElement(solRowIndex, solColIndex, getElement(i, j));
			}
		}
		return solMat;
	}

	// example of a vector for a 2by2 matrix: [Mat[0,0] Mat[0,1], Mat[1,0], Mat[1,1]]
	void MatrixFromVector(vector<T>& initializeVec) {
		if (initializeVec.size() != rows_ * cols_) throw MatrixExeption("vector should be able to fill the whole matrix");
		int i = 1, j = 1;
		typename vector<T>::iterator it = initializeVec.begin();
		for (i = 1; i <= rows_; i++) {
			for (j = 1; j <= cols_; j++) {
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

	Matrix& operator=(Matrix<T> srcMat) {
		if (rows_ < srcMat.getRowsNum() || cols_ < srcMat.getColumnsNum()) throw MatrixExeption("error in assign operator: solMat should be able to fill source mat");
		if (rows_ != srcMat.getRowsNum() || cols_ != srcMat.getColumnsNum()) {
			delete matrix_;
			matrix_ = new T[srcMat.getRowsNum() * srcMat.getColumnsNum()];
			rows_ = srcMat.getRowsNum();
			cols_ = srcMat.getColumnsNum();
		}
		int i = 1, j = 1;
		for (i = 1; i <= srcMat.getRowsNum(); i++) {
			for (j = 1; j <= srcMat.getColumnsNum(); j++) {
				setElement(i, j, srcMat.getElement(i, j));
			}
		}
		return *this;
	}


	//overloading operator =
	/*
	Matrix& operator=(Matrix<T>& srcMat) {
		if (rows_ < srcMat.getRowsNum() || cols_ < srcMat.getColumnsNum()) throw MatrixExeption("error in assign operator: solMat should be able to fill source mat");
		int i = 1, j = 1;
		for (i = 1; i <= srcMat.getRowsNum(); i++) {
			for (j = 1; j <= srcMat.getColumnsNum(); j++) {
				setElement(i, j, srcMat.getElement(i, j));
			}
		}
		return *this;
	}
	*/

	Matrix& operator=(vector<T>& srcVec) {
		MatrixFromVector(srcVec);
		return *this;
	}

	void printMatrix() {
		int i = 1, j = 1;
		for (i = 1; i <= rows_; i++) {
			for (j = 1; j <= cols_; j++) {
				cout << getElement(i, j) << " ";
			}
			cout << endl;
		}
		cout << endl;
	}

	// get the transpose of "this" matrix
	Matrix<T> getTranspose() {
		Matrix<T> solMat(getColumnsNum(), getRowsNum());
		int i = 1, j = 1;
		T elem;
		for (i = 1; i <= rows_; i++) {
			for (j = 1; j <= cols_; j++) {
				elem = getElement(i, j);
				solMat.setElement(j, i, elem);
			}
		}
		return solMat;
	}

	// this function multiply "this" matrix with another one
	Matrix<T> operator*(Matrix<T>& MatB) {
		if (cols_ != MatB.getRowsNum()) throw MatrixExeption("first matrix rows not equal to second matrix cols");
		Matrix<T> solMat(rows_, MatB.getColumnsNum());
		int i = 1, j = 1, currRow = 1;
		T solElem = T(), MatAelem = T(), MatBelem = T();
		for (i = 1; i <= rows_; i++) {
			for (j = 1; j <= MatB.getColumnsNum(); j++) {
				for (currRow = 1; currRow <= MatB.getRowsNum(); currRow++) {
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

	Matrix<T> operator*(vector<T>& srcVector) {
		Matrix<T> solMat;
		return solMat;
	}

	// adds 2 matrixes
	Matrix<T> operator+(Matrix<T>& MatB) {
		if (cols_ != MatB.getColumnsNum() || rows_ != MatB.getRowsNum()) throw MatrixExeption("both matrixes should have the same number of rows and cols");
		Matrix<T> solMat(rows_, cols_);
		int i = 1, j = 1;
		T solElem, MatAelem, MatBelem;
		for (i = 1; i <= rows_; i++) {
			for (j = 1; j <= cols_; j++) {
				MatAelem = getElement(i, j);
				MatBelem = MatB.getElement(i, j);
				solElem = MatAelem + MatBelem;
				solMat.setElement(i, j, solElem);
			}
		}
		return solMat;
	}

	// substract 2 matrixes
	Matrix<T> operator-(Matrix<T>& MatB) {
		if (cols_ != MatB.getColumnsNum() || rows_ != MatB.getRowsNum()) throw MatrixExeption("both matrixes should have the same number of rows and cols");
		Matrix<T> solMat(rows_, cols_);
		int i = 1, j = 1;
		T solElem, MatAelem, MatBelem;
		for (i = 1; i <= rows_; i++) {
			for (j = 1; j <= cols_; j++) {
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


