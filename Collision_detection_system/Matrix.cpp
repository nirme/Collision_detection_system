#include "Matrix.h"


Matrix::Matrix(unsigned int _rows, unsigned int _columns, float fill) :
	data(_rows*_columns, fill),
	rows(_rows), 
	columns(_columns)
{};


Matrix::Matrix(const Matrix& rhs) :
	rows(rhs.rows),
	columns(rhs.columns),
	data(rhs.data)
{};


Matrix::Matrix(Matrix&& rhs) :
	rows(rhs.rows),
	columns(rhs.columns),
	data(rhs.data)
{};


Matrix& Matrix::operator= (const Matrix& rhs)
{
	rows = rhs.rows;
	columns = rhs.columns;
	data.assign(rhs.data.begin(), rhs.data.end());
	return *this;
};


Matrix& Matrix::operator= (Matrix&& rhs)
{
	std::swap(rows, rhs.rows);
	std::swap(columns, rhs.columns);
	std::swap(data, rhs.data);
	return *this;
};


unsigned int Matrix::getRows() const
{
	return rows;
};


unsigned int Matrix::getColumns() const
{
	return columns;
};


void Matrix::resize(unsigned int _rows, unsigned int _columns, float _fill)
{
	rows = _rows;
	columns = _columns;

	// reserve first to get exact length, then resize to fill without changing capacity
	data.reserve(rows * columns);
	data.resize(rows * columns, _fill);
	data.shrink_to_fit();
};


float Matrix::operator()(unsigned int i, unsigned int j) const
{
	assert(i < rows && j < columns && "index out of scope");
	return data[columns * i + j];
};


float& Matrix::operator()(unsigned int i, unsigned int j)
{
	assert(i < rows && j < columns && "index out of scope");
	return data[columns * i + j];
};


Matrix& Matrix::fill(unsigned int i, unsigned int j, float* p, unsigned int r, unsigned int c)
{
	assert(i + r < rows &&	j + c < columns && "index out of scope");

	for (unsigned int i2 = 0; i2 < r; ++i2)
		for (unsigned int j2 = 0; j2 < c; ++j2)
			data[(i + i2)*columns + (j + j2)] = p[i2*columns + j2];

	return *this;
};


void Matrix::transpose()
{
	unsigned int l = (unsigned int)data.size();
	std::vector<float> newData(l);

	for (unsigned int i = 0; i < l; ++i)
		newData[(i%columns)*columns + i / rows] = data[i];

	std::swap(data, newData);
	std::swap(rows, columns);
};

/*
Matrix& Matrix::inverse()
{
	assert(rows == columns && "matrix is uninversible");

	// diagonal check
	for (unsigned int i = 0; i < rows; ++i)
		for (unsigned int j = i + 1; j < rows; ++j)
			if (data[i*columns + j] != 0.0f)
				goto non_diagonal;

	for (unsigned int i = 0; i < rows; ++i)
		if (data[i*columns + i] == 0.0f)
			goto non_diagonal;

	{
	diagonal:

		for (unsigned int i = 0; i < rows; ++i)
			data[i*columns + i] = 1.0f / data[i*columns + i];

		return *this;
	}


	{
	non_diagonal:

		assert(1 != 0 && "unimplemented");
		return *this;
	}
};
*/

Matrix& Matrix::inverse_diagonal()
{
	assert(rows == columns && "matrix is uninversible");

	// diagonal check
	for (unsigned int i = 0; i < rows; ++i)
		for (unsigned int j = i + 1; j < rows; ++j)
			assert(data[i*columns + j] == 0.0f && "matrix not diagonal");

	for (unsigned int i = 0; i < rows; ++i)
		assert(data[i*columns + i] != 0.0f && "matrix not diagonal");

	for (unsigned int i = 0; i < rows; ++i)
		data[i*columns + i] = 1.0f / data[i*columns + i];

	return *this;
};



Matrix operator* (const Matrix& m1, const Matrix& m2)
{
	assert(m1.getColumns() == m2.getRows() && "index out of scope");

	unsigned int r = m1.getRows(), c = m2.getColumns();
	Matrix out(r, c);

	unsigned int ri, ci, l = m1.getColumns();

	for (unsigned int i = 0, iMax = r * c; i < iMax; ++i)
	{
		ri = i / c;
		ci = i % c;

		float &val = out(ri, ci);
		val = 0.0f;

		for (unsigned int j = 0; j < l; ++j)
			val += m1(ri, j) * m2(j, ci);
	}

	return out;
};


Matrix solveGaussSeidel(const Matrix& A, const Matrix& b)
{
	unsigned int r = A.getRows();
	Matrix x(r, 1, 1.0f);
	float diff = P_INF;

	while (diff > 1.0f)
	{
		diff = 0.0f;

		for (unsigned int i = 0; i < r; ++i)
		{
			// Ux(t)
			float ux = 0;
			for (unsigned int j = i + 1; j < r; ++j)
				ux += A(i, j)*x(j, 0);

			//Lx(t+1)
			float lx = 0.0f;
			for (unsigned int j = 0; j < i; ++j)
				lx += A(i, j)*x(j, 0);

			// b - Ux(t) - Lx(t+1)
			float xi = (b(i, 0) - ux - lx) / A(i, i);

			float df = std::abs(xi - x(i, 0));
			diff = df > diff ? df : diff;
			x(i, 0) = xi;
		}
	}

	return x;
};


