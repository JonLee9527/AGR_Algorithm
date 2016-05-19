#ifndef DJI_ALGRICULTURE_DJI_MAT_H_H
#define DJI_ALGRICULTURE_DJI_MAT_H_H

#include <stdlib.h>
#include <stdio.h>


/*@brief Self-defined class to replace opencv Mat
@brief dim=2, channels=1, depth =64
*/
class MatD
{
public:
	MatD();
	MatD(const MatD& mat);
	MatD(int rows, int cols, double* data);
	MatD(int rows, int cols, double val = 0.0);
	MatD& operator= (const MatD& mat);
	~MatD()
	{
		if (data!=NULL)
		{
			delete[]data;
		}
	}
public:
	double* data;

	int rows, cols;

	MatD inv()const;

	MatD mul(const MatD & mat)const;

	MatD t()const;

	void print() const;
	int step()const
	{
		return rows * 8;
	}

public:

	static MatD unitMat(int rows, int cols);

public: //operator
	const double& operator () (int row, int col) const;
	double& operator () (int row, int col);

	MatD operator + (const MatD& mat) const;
	MatD operator - (const MatD& mat) const;
	MatD operator * (const MatD& mat)const;
	MatD operator * (double s)const;
	MatD operator / (double s)const;
	

};

MatD operator * (double s,const MatD& mat);

#endif