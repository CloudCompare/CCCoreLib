// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © CloudCompare Project

#pragma once

//Local
#include "SquareMatrix.h"

namespace CCCoreLib
{
	//! Jacobi eigen vectors/values decomposition
	template <typename Scalar> class Jacobi
	{
	public:
		
		using SquareMatrix = SquareMatrixTpl<Scalar>;
		using EigenValues = std::vector<Scalar>;

		//! Tests tha consistency between eigenvalues and eigenvectors
		static bool TestEigenVecAndValues(const SquareMatrix& matrix, const SquareMatrix& eigenVectors, const EigenValues& eigenValues)
		{
			CCVector3 e1(	static_cast<PointCoordinateType>(eigenVectors.getValue(0, 0)),
							static_cast<PointCoordinateType>(eigenVectors.getValue(1, 0)),
							static_cast<PointCoordinateType>(eigenVectors.getValue(2, 0)) );
			CCVector3 e2(	static_cast<PointCoordinateType>(eigenVectors.getValue(0, 1)),
							static_cast<PointCoordinateType>(eigenVectors.getValue(1, 1)),
							static_cast<PointCoordinateType>(eigenVectors.getValue(2, 1)) );
			CCVector3 e3(	static_cast<PointCoordinateType>(eigenVectors.getValue(0, 2)),
							static_cast<PointCoordinateType>(eigenVectors.getValue(1, 2)),
							static_cast<PointCoordinateType>(eigenVectors.getValue(2, 2)) );

			CCVector3 imE1, imE2, imE3;
			matrix.apply(e1.u, imE1.u);
			matrix.apply(e2.u, imE2.u);
			matrix.apply(e3.u, imE3.u);

			CCVector3 l1_e1 = static_cast<PointCoordinateType>(eigenValues[0]) * e1;
			CCVector3 l2_e2 = static_cast<PointCoordinateType>(eigenValues[1]) * e2;
			CCVector3 l3_e3 = static_cast<PointCoordinateType>(eigenValues[2]) * e3;

			PointCoordinateType v1 = e1.dot(imE1) / e1.norm();	// should equal v1
			PointCoordinateType dot1 = v1 / imE1.norm();		// should equal 1

			PointCoordinateType v2 = e2.dot(imE2) / e2.norm();	// should equal v2
			PointCoordinateType dot2 = v2 / imE2.norm();		// should equal 1

			PointCoordinateType v3 = e3.dot(imE3) / e3.norm();	// should equal v3
			PointCoordinateType dot3 = v3 / imE3.norm();		// should equal 1

			CCVector3 res1 = l1_e1 - imE1; // should equal (0, 0, 0)
			CCVector3 res2 = l2_e2 - imE2; // should equal (0, 0, 0)
			CCVector3 res3 = l3_e3 - imE3; // should equal (0, 0, 0)

			PointCoordinateType norm1 = res1.norm(); // should equal 0
			PointCoordinateType norm2 = res2.norm(); // should equal 0
			PointCoordinateType norm3 = res3.norm(); // should equal 0

			if (	abs(norm1) > 1.0e-5
				||	abs(norm2) > 1.0e-5
				||	abs(norm3) > 1.0e-5 )
			{
				return false;
			}

			return true;
		}
		
		#define ROTATE(a,i,j,k,l) { Scalar g = a[i][j]; h = a[k][l]; a[i][j] = g-s*(h+g*tau); a[k][l] = h+s*(g-h*tau); }

		//! Computes the eigenvalues and eigenvectors of a given symmetrical square matrix
		/** Uses the Jacobian method.
			See the Numerical Recipes.

			\warning Input matrix is diagonilized in place.

			\param[in]  matrix            symmetrical square matrix
			\param[out] eigenVectors      output eigenvectors (as a square matrix)
			\param[out] eigenValues       output eigenvalues
			\param[in]  absoluteValues    whether to output absolute eigenvalues only
			\param[in]  maxIterationCount max number of iteration (optional)
			\return success
		**/
		static bool ComputeEigenValuesAndVectors(	SquareMatrix& matrix,
													SquareMatrix& eigenVectors,
													EigenValues& eigenValues,
													bool absoluteValues = true,
													unsigned maxIterationCount = 50)
		{
			if (!matrix.isValid())
			{
				return false;
			}
			
			unsigned n = matrix.size();
			unsigned matrixSquareSize = n * n;
			
			//Output eigen vectors matrix
			eigenVectors = SquareMatrix(n);
			if (!eigenVectors.isValid())
			{
				return false;
			}
			eigenVectors.toIdentity();
			
			std::vector<Scalar> b, z;
			try
			{
				eigenValues.resize(n);
				b.resize(n);
				z.resize(n);
			}
			catch (const std::bad_alloc&)
			{
				//Not enough memory
				return false;
			}
			Scalar* d = eigenValues.data();
			
			//Initialization
			{
				for (unsigned ip = 0; ip < n; ip++)
				{
					b[ip] = d[ip] = matrix.m_values[ip][ip]; //Initialize b and d to the diagonal of a.
					z[ip] = 0; //This vector will accumulate terms of the form tapq as in equation (11.1.14)
				}
			}
			
			for (unsigned i = 1; i <= maxIterationCount; i++)
			{
				//Sum off-diagonal elements
				Scalar sm = 0;
				{
					for (unsigned ip = 0; ip < n - 1; ip++)
					{
						for (unsigned iq = ip + 1; iq < n; iq++)
						{
							sm += std::abs(matrix.m_values[ip][iq]);
						}
					}
				}
				
				if (sm == 0) //The normal return, which relies on quadratic convergence to machine underflow.
				{
					if (absoluteValues)
					{
						//We only need the absolute values of eigenvalues
						for (unsigned ip = 0; ip < n; ip++)
						{
							d[ip] = std::abs(d[ip]);
						}
					}

					//Consistency test
					//assert(TestEigenVecAndValues(matrix, eigenVectors, eigenValues));
					
					return true;
				}
				
				Scalar tresh = 0;
				if (i < 4)
				{
					tresh = sm / static_cast<Scalar>(5 * matrixSquareSize); //...on the first three sweeps.
				}
				
				for (unsigned ip = 0; ip < n - 1; ip++)
				{
					for (unsigned iq = ip + 1; iq < n; iq++)
					{
						Scalar pq = std::abs(matrix.m_values[ip][iq]) * 100;
						//After four sweeps, skip the rotation if the off-diagonal element is small.
						if (	i > 4
							&&	static_cast<float>(std::abs(d[ip]) + pq) == static_cast<float>(std::abs(d[ip]))
							&&	static_cast<float>(std::abs(d[iq]) + pq) == static_cast<float>(std::abs(d[iq])))
						{
							matrix.m_values[ip][iq] = 0;
						}
						else if (std::abs(matrix.m_values[ip][iq]) > tresh)
						{
							Scalar h = d[iq] - d[ip];
							Scalar t = 0;
							if (static_cast<float>(std::abs(h) + pq) == static_cast<float>(std::abs(h)))
							{
								t = matrix.m_values[ip][iq] / h;
							}
							else
							{
								Scalar theta = h / (2 * matrix.m_values[ip][iq]); //Equation (11.1.10).
								t = 1 / (std::abs(theta) + sqrt(1 + theta*theta));
								if (theta < 0)
								{
									t = -t;
								}
							}
							
							Scalar c = 1 / sqrt(t*t + 1);
							Scalar s = t*c;
							Scalar tau = s / (1 + c);
							h = t * matrix.m_values[ip][iq];
							z[ip] -= h;
							z[iq] += h;
							d[ip] -= h;
							d[iq] += h;
							matrix.m_values[ip][iq] = 0;
							
							//Case of rotations 1 <= j < p
							{
								for (unsigned j = 0; j + 1 <= ip; j++)
								{
									ROTATE(matrix.m_values, j, ip, j, iq)
								}
							}
							//Case of rotations p < j < q
							{
								for (unsigned j = ip + 1; j + 1 <= iq; j++)
								{
									ROTATE(matrix.m_values, ip, j, j, iq)
								}
							}
							//Case of rotations q < j <= n
							{
								for (unsigned j = iq + 1; j < n; j++)
								{
									ROTATE(matrix.m_values, ip, j, iq, j)
								}
							}
							//Last case
							{
								for (unsigned j = 0; j < n; j++)
								{
									ROTATE(eigenVectors.m_values, j, ip, j, iq)
								}
							}
						}
					}
				}
				
				//Update b, d and z
				{
					for (unsigned ip = 0; ip < n; ip++)
					{
						b[ip] += z[ip];
						d[ip] = b[ip];
						z[ip] = 0;
					}
				}
			}
			
			//Too many iterations!
			return false;
		}
		
		//! Sorts the eigenvectors in the decreasing order of their associated eigenvalues (in place)
		/** \param[in,out] eigenVectors eigenvectors (as a square matrix)
			\param[in,out] eigenValues  eigenvalues
			\return success
		**/
		static bool SortEigenValuesAndVectors(SquareMatrix& eigenVectors, EigenValues& eigenValues)
		{
			unsigned n = eigenVectors.size();

			if (	!eigenVectors.isValid()
				||	n < 2
				||	n != eigenValues.size())
			{
				assert(false);
				return false;
			}

			for (unsigned i = 0; i < n - 1; i++)
			{
				unsigned maxValIndex = i;
				for (unsigned j = i + 1; j < n; j++)
				{
					if (eigenValues[j] > eigenValues[maxValIndex])
					{
						maxValIndex = j;
					}
				}

				if (maxValIndex != i)
				{
					std::swap(eigenValues[i], eigenValues[maxValIndex]);
					for (unsigned j = 0; j < n; ++j)
					{
						std::swap(eigenVectors.m_values[j][i], eigenVectors.m_values[j][maxValIndex]);
					}
				}
			}
			
			return true;
		}
		
		//! Returns a specific eigenvector
		/** \param[in]  eigenVectors eigenvectors (as a square matrix)
			\param[in]  index        requested eigenvector index (< eigenvectors matrix size)
			\param[out] eigenVector  output vector (same size as the matrix)
			\return success
		**/
		static bool GetEigenVector(const SquareMatrix& eigenVectors, unsigned index, Scalar eigenVector[])
		{
			unsigned n = eigenVectors.size();

			if (eigenVector && index < n)
			{
				for (size_t i = 0; i < n; ++i)
				{
					eigenVector[i] = eigenVectors.m_values[i][index];
				}
				return true;
			}
			else
			{
				assert(false);
				return false;
			}
		}
		
		//! Returns the biggest eigenvalue and its associated eigenvector
		/** \param[in]  eigenVectors   eigenvectors (as a square matrix)
			\param[in]  eigenValues    eigenvalues
			\param[out] maxEigenValue  biggest eigenvalue
			\param[out] maxEigenVector eigenvector vector corresponding to the biggest eigenvalue
			\return success
		**/
		static bool GetMaxEigenValueAndVector(	const SquareMatrix& eigenVectors,
												const EigenValues& eigenValues,
												Scalar& maxEigenValue,
												Scalar maxEigenVector[])
		{
			unsigned n = eigenVectors.size();

			if (	!eigenVectors.isValid()
				||	n < 2
				||	n != eigenValues.size())
			{
				assert(false);
				return false;
			}
			
			unsigned maxIndex = 0;
			for (unsigned i = 1; i < n; ++i)
			{
				if (eigenValues[i] > eigenValues[maxIndex])
				{
					maxIndex = i;
				}
			}
			
			maxEigenValue = eigenValues[maxIndex];
			return GetEigenVector(eigenVectors, maxIndex, maxEigenVector);
		}
		
		//! Returns the smallest eigenvalue and its associated eigenvector
		/** \param[in]  eigenVectors   eigenvectors (as a square matrix)
			\param[in]  eigenValues    eigenvalues
			\param[out] minEigenValue  smallest eigenvalue
			\param[out] minEigenVector eigenvector vector corresponding to the smallest eigenvalue
			\return success
		**/
		static bool GetMinEigenValueAndVector(	const SquareMatrix& eigenVectors,
												const EigenValues& eigenValues,
												Scalar& minEigenValue,
												Scalar minEigenVector[])
		{
			unsigned n = eigenVectors.size();

			if (	!eigenVectors.isValid()
				||	n < 2
				||	n != eigenValues.size())
			{
				assert(false);
				return false;
			}

			unsigned minIndex = 0;
			for (unsigned i = 1; i < n; ++i)
			{
				if (eigenValues[i] < eigenValues[minIndex])
				{
					minIndex = i;
				}
			}

			minEigenValue = eigenValues[minIndex];
			return GetEigenVector(eigenVectors, minIndex, minEigenVector);
		}
	};
}
