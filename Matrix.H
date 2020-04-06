// defining a generic matrix

#ifndef MATRIX_H
#define MATRIX_H

template <class T>
class Mat {
public:
	Mat(int rows, int cols) : cols_(cols), rows_(rows) {
		int i;
		T* arr;
		arr = new T[rows * cols];
		for (i = 0; i < rows * cols; i++) {
			arr[i] = 0;
		}
		matrix_ = arr;
	}
	int getRowsNum() const {
		return rows_;
	}
	int getColumnsNum() const {
		return cols_;
	}
	T getElement(int row, int col) const {
		return matrix_[row * cols_ + col];
	}
	void setElement(int row, int col, T elem) {
		matrix_[row * cols_ + col] = elem;
	}
	
	~Mat() {
		delete []matrix_;
	}
protected:
	int rows_;
	int cols_;
	T* matrix_;
};



#endif
