#pragma once

#include "Constants.h"
#include <vector>
#include <cassert>

class Matrix
{
private:
	std::vector<float> data;
	unsigned int rows, columns;

public:

	Matrix(unsigned int _rows = 0, unsigned int _columns = 0, float fill = 0.0f);
	Matrix(const Matrix& rhs);
	Matrix(Matrix&& rhs);

	Matrix& operator= (const Matrix& rhs);
	Matrix& operator= (Matrix&& rhs);

	unsigned int getRows() const;
	unsigned int getColumns() const;

	void resize(unsigned int _rows, unsigned int _columns, float _fill = 0.0f);

	float operator()(unsigned int i, unsigned int j) const;
	float& operator()(unsigned int i, unsigned int j);

	Matrix& fill(unsigned int i, unsigned int j, float* p, unsigned int r, unsigned int c);

	void transpose();

	//Matrix& inverse();
	Matrix& inverse_diagonal();
};


Matrix operator* (const Matrix& m1, const Matrix& m2);

Matrix solveGaussSeidel(const Matrix& A, const Matrix& b);
