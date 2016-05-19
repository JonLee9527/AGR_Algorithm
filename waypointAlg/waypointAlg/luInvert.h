#ifndef DJI_ALGRICULTURE_DJI_MAT_LU_INVERT_H_H
#define DJI_ALGRICULTURE_DJI_MAT_LU_INVERT_H_H


#include "DjiMatD.h"
#include "assert.h"

//**********************invert LU**********************//

#define  double_bytes 8

#define det2(m)   ((double)m(0,0)*m(1,1) - (double)m(0,1)*m(1,0))

#define det3(m)   (m(0,0)*((double)m(1,1)*m(2,2) - (double)m(1,2)*m(2,1)) -  \
	m(0, 1)*((double)m(1, 0)*m(2, 2) - (double)m(1, 2)*m(2, 0)) + \
	m(0, 2)*((double)m(1, 0)*m(2, 1) - (double)m(1, 1)*m(2, 0)))

template<typename _Tp> static inline int
LUImpl(_Tp* A, size_t astep, int m, _Tp* b, size_t bstep, int n)
{
	int i, j, k, p = 1;
	astep /= sizeof(A[0]);
	bstep /= sizeof(b[0]);

	for (i = 0; i < m; i++)
	{
		k = i;

		for (j = i + 1; j < m; j++)
		if (std::abs(A[j*astep + i]) > std::abs(A[k*astep + i]))
			k = j;

		if (std::abs(A[k*astep + i]) < std::numeric_limits<_Tp>::epsilon())
			return 0;

		if (k != i)
		{
			for (j = i; j < m; j++)
				std::swap(A[i*astep + j], A[k*astep + j]);
			if (b)
			for (j = 0; j < n; j++)
				std::swap(b[i*bstep + j], b[k*bstep + j]);
			p = -p;
		}

		_Tp d = -1 / A[i*astep + i];

		for (j = i + 1; j < m; j++)
		{
			_Tp alpha = A[j*astep + i] * d;

			for (k = i + 1; k < m; k++)
				A[j*astep + k] += alpha*A[i*astep + k];

			if (b)
			for (k = 0; k < n; k++)
				b[j*bstep + k] += alpha*b[i*bstep + k];
		}

		A[i*astep + i] = -d;
	}

	if (b)
	{
		for (i = m - 1; i >= 0; i--)
		for (j = 0; j < n; j++)
		{
			_Tp s = b[i*bstep + j];
			for (k = i + 1; k < m; k++)
				s -= A[i*astep + k] * b[k*bstep + j];
			b[i*bstep + j] = s*A[i*astep + i];
		}
	}

	return p;
}

static int LU(double* A, size_t astep, int m, double* b, size_t bstep, int n)
{
	return LUImpl(A, astep, m, b, bstep, n);
}

//use LU method
static double dji_invert(const MatD src, MatD& dst)
{
	bool result = false;

	dst = MatD::unitMat(src.rows, src.cols);
	if (src.rows <= 3)
	{
		double* srcdata = src.data;
		double* dstdata = dst.data;

		if (src.rows == 2)
		{
			double d = det2(src);
			if (d != 0.)
			{
				double t0, t1;
				result = true;
				d = 1. / d;
				t0 = src(0, 0)*d;
				t1 = src(1, 1)*d;
				dst(1, 1) = t0;
				dst(0, 0) = t1;
				t0 = -src(0, 1)*d;
				t1 = -src(1, 0)*d;
				dst(0, 1) = t0;
				dst(1, 0) = t1;
			}
		}
		else if (src.rows == 3)
		{
			double d = det3(src);
			if (d != 0.)
			{
				double t[9];
				result = true;
				d = 1. / d;

				t[0] = (src(1, 1) * src(2, 2) - src(1, 2) * src(2, 1)) * d;
				t[1] = (src(0, 2) * src(2, 1) - src(0, 1) * src(2, 2)) * d;
				t[2] = (src(0, 1) * src(1, 2) - src(0, 2) * src(1, 1)) * d;

				t[3] = (src(1, 2) * src(2, 0) - src(1, 0) * src(2, 2)) * d;
				t[4] = (src(0, 0) * src(2, 2) - src(0, 2) * src(2, 0)) * d;
				t[5] = (src(0, 2) * src(1, 0) - src(0, 0) * src(1, 2)) * d;

				t[6] = (src(1, 0) * src(2, 1) - src(1, 1) * src(2, 0)) * d;
				t[7] = (src(0, 1) * src(2, 0) - src(0, 0) * src(2, 1)) * d;
				t[8] = (src(0, 0) * src(1, 1) - src(0, 1) * src(1, 0)) * d;

				dst(0, 0) = t[0]; dst(0, 1) = t[1]; dst(0, 2) = t[2];
				dst(1, 0) = t[3]; dst(1, 1) = t[4]; dst(1, 2) = t[5];
				dst(2, 0) = t[6]; dst(2, 1) = t[7]; dst(2, 2) = t[8];
			}
		}
		else //src.rows ==0 || src.rows==1
		{
			assert(src.rows == 1);

			double d = src(0, 0);
			if (d != 0.)
			{
				result = true;
				dst(0, 0) = 1. / d;
			}
		}
		return result;
	}
	int n = dst.cols;

	// 	cout << "src:" << endl;
	// 	src.print();
	// 	cout << "dst:" << endl;
	// 	dst.print();

	result = LU((double*)src.data, n*double_bytes, n, (double*)dst.data, n*double_bytes, n) != 0;

	return result;
}



#endif // !DJI_ALGRICULTURE_DJI_MAT_LU_INVERT_H_H
