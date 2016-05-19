#include "stdafx.h"

#include "assert.h"

#include <algorithm>

#include "DjiMatD.h"

#include "debugIO.h"

#include "luInvert.h"

#include "djiUtils.h"

using namespace std;


MatD::MatD()
{
	rows = 0;
	cols = 0;
	data = NULL;
		
}
MatD::MatD(int _rows, int _cols, double* _data)
{
	rows = _rows;
	cols = _cols;
	data = new double[rows*cols];
	for (int i = 0; i < rows*cols; i++)
		data[i] = _data[i];
}
MatD::MatD(int _rows, int _cols, double _val)
{
	rows = _rows;
	cols = _cols;
	data = new double[rows*cols];
	for (int i = 0; i < rows*cols; i++)
		data[i] = _val;
}
MatD::MatD(const MatD& mat)
{
	rows = mat.rows;
	cols = mat.cols;
	data = new double[rows*cols];
	for (int i = 0; i < rows*cols; i++)
		data[i] = mat.data[i];
}
MatD& MatD::operator= (const MatD& mat)
{
	if (this == &mat)
	{
		return *this;
	}
	else
	{
		delete[] data;
		data = new double[mat.rows*mat.cols];
		rows = mat.rows;
		cols = mat.cols;
		for (int i = 0; i < rows*cols; i++)
			data[i] = mat.data[i];
		return *this;
	}
}


const double& MatD::operator () (int row, int col) const
{
	return data[row*cols + col];
}

double& MatD::operator () (int row, int col)
{
	return data[row*cols + col];
}

MatD MatD::operator + (const MatD& mat)const
{
	if (rows != mat.rows || cols != mat.cols)
		return MatD();
	MatD dst = mat;
	for (int i = 0; i < dst.rows;i++)
	{
		for (int j = 0; j < dst.cols;j++)
		{
			dst(i, j) = (*this)(i, j) + mat(i, j);
		}
	}
	return dst;
}
MatD MatD::operator - (const MatD& mat)const
{
	if (rows != mat.rows || cols != mat.cols)
		return MatD();
	MatD dst = mat;
	for (int i = 0; i < dst.rows; i++)
	{
		for (int j = 0; j < dst.cols; j++)
		{
			dst(i, j) = (*this)(i, j) - mat(i, j);
		}
	}
	return dst;
}
MatD MatD::operator * (const MatD& mat)const
{
	return mul(mat);
}
MatD MatD::operator * (double s)const
{
	MatD dst = (*this);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			dst(i, j) *= s;
		}
	}
	return dst;
}
MatD operator * (double s, const MatD& mat)
{
	return mat*s;
}
MatD MatD::operator / (double s)const
{
	MatD dst = (*this);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			dst(i, j) /= s;
		}
	}
	return dst;
}

MatD MatD::inv()const
{
	MatD dst;
	dji_invert(*this, dst);
	return dst;
}

MatD MatD::t()const
{
	MatD dst = MatD(cols,rows);
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			dst(j, i) = (*this)(i, j);
		}
	}
	return dst;
}

void MatD::print()const
{
	for (int i = 0; i < rows*cols;i++)
	{
		printLog(double2string(data[i])+", ");
	}
	printLog(" ");
}

MatD MatD::unitMat(int rows, int cols)
{
	if (rows != cols)
	{
		printLog("input Error in MatD::ones", true);
		return MatD();
	}
		
	MatD mat = MatD(rows, cols, 0.0);
	for (int i = 0; i < rows;i++)
	{
		mat(i, i) = 1.0;
	}
	return mat;
}


MatD MatD::mul(const MatD &mat)const
{
	if (cols != mat.rows)
		return MatD();
	MatD dst(rows, mat.cols);
	for (int i = 0; i < dst.rows;i++)
	{
		for (int j = 0; j < dst.cols;j++)
		{
			dst(i, j) = 0.0;
			for (int k = 0; k < cols;k++)
			{
				dst(i, j) += (*this)(i, k)*mat(k,j);
			}
		}
	}
	return dst;
}







