// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include <Neighbourhood.h>

//local
#include <CCMath.h>
#include <ConjugateGradient.h>
#include <Delaunay2dMesh.h>
#include <DistanceComputationTools.h>
#include <Jacobi.h>
#include <PointCloud.h>
#include <SimpleMesh.h>

//System
#include <algorithm>


using namespace CCCoreLib;

Neighbourhood::Neighbourhood(GenericIndexedCloudPersist* associatedCloud)
	: m_globalQuadricEquation{ 0, 0, 0, 0, 0, 0 }
	, m_quadricEquationDirections(0, 1, 2)
	, m_lsPlaneEquation{ 0, 0, 0, 0 }
	, m_structuresValidity(FLAG_DEPRECATED)
	, m_associatedCloud(associatedCloud)
{
	assert(m_associatedCloud);
}

void Neighbourhood::reset()
{
	m_structuresValidity = FLAG_DEPRECATED;
}

const CCVector3* Neighbourhood::getLocalGravityCenter()
{
	if (!(m_structuresValidity & FLAG_GRAVITY_CENTER))
	{
		computeLocalGravityCenter();
	}
	return ((m_structuresValidity & FLAG_GRAVITY_CENTER) ? &m_localGravityCenter : nullptr);
}

void Neighbourhood::setLocalGravityCenter(const CCVector3& G)
{
	m_localGravityCenter = G;
	m_structuresValidity |= FLAG_GRAVITY_CENTER;
}

const PointCoordinateType* Neighbourhood::getLSPlane()
{
	if (!(m_structuresValidity & FLAG_LS_PLANE))
	{
		computeLeastSquareBestFittingPlane();
	}
	return ((m_structuresValidity & FLAG_LS_PLANE) ? m_lsPlaneEquation : nullptr);
}

void Neighbourhood::setLSPlane(	const PointCoordinateType eq[4],
								const CCVector3& X,
								const CCVector3& Y,
								const CCVector3& N )
{
	memcpy(m_lsPlaneEquation, eq, sizeof(PointCoordinateType)*4);
	m_lsPlaneVectors[0] = X;
	m_lsPlaneVectors[1] = Y;
	m_lsPlaneVectors[2] = N;

	m_structuresValidity |= FLAG_LS_PLANE;
}

const CCVector3* Neighbourhood::getLSPlaneX()
{
	if (!(m_structuresValidity & FLAG_LS_PLANE))
	{
		computeLeastSquareBestFittingPlane();
	}
	return ((m_structuresValidity & FLAG_LS_PLANE) ? m_lsPlaneVectors : nullptr);
}

const CCVector3* Neighbourhood::getLSPlaneY()
{
	if (!(m_structuresValidity & FLAG_LS_PLANE))
	{
		computeLeastSquareBestFittingPlane();
	}
	return ((m_structuresValidity & FLAG_LS_PLANE) ? m_lsPlaneVectors + 1 : nullptr);
}

const CCVector3* Neighbourhood::getLSPlaneNormal()
{
	if (!(m_structuresValidity & FLAG_LS_PLANE))
	{
		computeLeastSquareBestFittingPlane();
	}
	return ((m_structuresValidity & FLAG_LS_PLANE) ? m_lsPlaneVectors + 2 : nullptr);
}

const PointCoordinateType* Neighbourhood::getLocalQuadric(Tuple3ub* dims/*=nullptr*/)
{
	if (!(m_structuresValidity & FLAG_QUADRIC))
	{
		computeQuadric();
	}

	if (dims)
	{
		*dims = m_quadricEquationDirections;
	}

	return ((m_structuresValidity & FLAG_QUADRIC) ? m_localQuadricEquation : nullptr);
}

const double* Neighbourhood::getGlobalQuadric(Tuple3ub* dims/*=nullptr*/)
{
	if (!(m_structuresValidity & FLAG_QUADRIC))
	{
		computeQuadric();
	}

	if (dims)
	{
		*dims = m_quadricEquationDirections;
	}

	return ((m_structuresValidity & FLAG_QUADRIC) ? m_globalQuadricEquation : nullptr);
}

bool Neighbourhood::computeLocalGravityCenter()
{
	//invalidate previous gravity center (if any)
	m_structuresValidity &= (~FLAG_GRAVITY_CENTER);

	assert(m_associatedCloud);
	unsigned count = (m_associatedCloud ? m_associatedCloud->size() : 0);
	if (count == 0)
	{
		assert(false);
		return false;
	}

	//sum
	CCVector3d Psum{ 0, 0, 0 };
	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3* Plocal = m_associatedCloud->getLocalPoint(i);
		Psum.x += Plocal->x;
		Psum.y += Plocal->y;
		Psum.z += Plocal->z;
	}

	setLocalGravityCenter( {	static_cast<PointCoordinateType>(Psum.x / count),
								static_cast<PointCoordinateType>(Psum.y / count),
								static_cast<PointCoordinateType>(Psum.z / count)
							} );

	return true;
}

SquareMatrixd Neighbourhood::computeCovarianceMatrix()
{
	assert(m_associatedCloud);
	unsigned count = (m_associatedCloud ? m_associatedCloud->size() : 0);
	if (count == 0)
	{
		return {};
	}

	//we get centroid
	const CCVector3* G = getLocalGravityCenter();
	assert(G);

	//we build up covariance matrix
	double mXX = 0.0;
	double mYY = 0.0;
	double mZZ = 0.0;
	double mXY = 0.0;
	double mXZ = 0.0;
	double mYZ = 0.0;

	for (unsigned i = 0; i < count; ++i)
	{
		const CCVector3 Plocal = *m_associatedCloud->getLocalPoint(i) - *G;

		mXX += static_cast<double>(Plocal.x)*Plocal.x;
		mYY += static_cast<double>(Plocal.y)*Plocal.y;
		mZZ += static_cast<double>(Plocal.z)*Plocal.z;
		mXY += static_cast<double>(Plocal.x)*Plocal.y;
		mXZ += static_cast<double>(Plocal.x)*Plocal.z;
		mYZ += static_cast<double>(Plocal.y)*Plocal.z;
	}

	//symmetry
	SquareMatrixd covMat(3);
	covMat.m_values[0][0] = mXX / count;
	covMat.m_values[1][1] = mYY / count;
	covMat.m_values[2][2] = mZZ / count;
	covMat.m_values[1][0] = covMat.m_values[0][1] = mXY / count;
	covMat.m_values[2][0] = covMat.m_values[0][2] = mXZ / count;
	covMat.m_values[2][1] = covMat.m_values[1][2] = mYZ / count;

	return covMat;
}

PointCoordinateType Neighbourhood::computeLargestRadius()
{
	assert(m_associatedCloud);
	unsigned pointCount = (m_associatedCloud ? m_associatedCloud->size() : 0);
	if (pointCount < 2)
		return 0;

	//get the centroid
	const CCVector3* G = getLocalGravityCenter();
	if (!G)
	{
		assert(false);
		return PC_NAN;
	}

	double maxSquareDist = 0;
	for (unsigned i = 0; i < pointCount; ++i)
	{
		const CCVector3* Plocal = m_associatedCloud->getLocalPoint(i);
		const double d2 = (*Plocal - *G).norm2();
		if (d2 > maxSquareDist)
		{
			maxSquareDist = d2;
		}
	}

	return static_cast<PointCoordinateType>(sqrt(maxSquareDist));
}

bool Neighbourhood::computeLeastSquareBestFittingPlane()
{
	//invalidate previous LS plane (if any)
	m_structuresValidity &= (~FLAG_LS_PLANE);

	assert(m_associatedCloud);
	unsigned pointCount = (m_associatedCloud ? m_associatedCloud->size() : 0);

	//we need at least 3 points to compute a plane
	static_assert(CC_LOCAL_MODEL_MIN_SIZE[LS] >= 3, "Invalid CC_LOCAL_MODEL_MIN_SIZE size");
	if (pointCount < CC_LOCAL_MODEL_MIN_SIZE[LS])
	{
		//not enough points!
		return false;
	}

	CCVector3 G{ 0, 0, 0 };
	if (pointCount > 3)
	{
		SquareMatrixd covMat = computeCovarianceMatrix();

		//we determine plane normal by computing the smallest eigen value of M = 1/n * S[(p-µ)*(p-µ)']
		SquareMatrixd eigVectors;
		std::vector<double> eigValues;
		if (!Jacobi<double>::ComputeEigenValuesAndVectors(covMat, eigVectors, eigValues, true))
		{
			//failed to compute the eigen values!
			return false;
		}

		//get normal
		{
			CCVector3d vec(0, 0, 1);
			double minEigValue = 0;
			//the smallest eigen vector corresponds to the "least square best fitting plane" normal
			Jacobi<double>::GetMinEigenValueAndVector(eigVectors, eigValues, minEigValue, vec.u);
			m_lsPlaneVectors[2] = vec.toPC();
		}

		//get also X (Y will be deduced by cross product, see below
		{
			CCVector3d vec;
			double maxEigValue = 0;
			Jacobi<double>::GetMaxEigenValueAndVector(eigVectors, eigValues, maxEigValue, vec.u);
			m_lsPlaneVectors[0] = vec.toPC();
		}

		//get the centroid (should already be up-to-date - see computeCovarianceMatrix)
		G = *getLocalGravityCenter();
	}
	else
	{
		//we simply compute the normal of the 3 points by cross product!
		const CCVector3* A = m_associatedCloud->getLocalPoint(0);
		const CCVector3* B = m_associatedCloud->getLocalPoint(1);
		const CCVector3* C = m_associatedCloud->getLocalPoint(2);

		//get X (AB by default) and Y (AC - will be updated later) and deduce N = X ^ Y
		m_lsPlaneVectors[0] = (*B - *A);
		m_lsPlaneVectors[1] = (*C - *A);
		m_lsPlaneVectors[2] = m_lsPlaneVectors[0].cross(m_lsPlaneVectors[1]);

		//the plane passes through any of the 3 points
		G = *A;
	}

	//make sure all vectors are unit!
	if (LessThanSquareEpsilon(m_lsPlaneVectors[2].norm2()))
	{
		//this means that the points are colinear!
		//m_lsPlaneVectors[2] = CCVector3(0, 0, 1); //any normal will do
		return false;
	}
	else
	{
		m_lsPlaneVectors[2].normalize();
	}
	//normalize X as well
	m_lsPlaneVectors[0].normalize();
	//and update Y
	m_lsPlaneVectors[1] = m_lsPlaneVectors[2].cross(m_lsPlaneVectors[0]);

	//deduce the proper equation
	m_lsPlaneEquation[0] = m_lsPlaneVectors[2].x;
	m_lsPlaneEquation[1] = m_lsPlaneVectors[2].y;
	m_lsPlaneEquation[2] = m_lsPlaneVectors[2].z;

	//eventually we just have to compute the 'constant' coefficient a3
	//we use the fact that the plane pass through G --> GM.N = 0 (scalar prod)
	//i.e. a0*G[0]+a1*G[1]+a2*G[2]=a3
	m_lsPlaneEquation[3] = G.dot(m_lsPlaneVectors[2]);

	m_structuresValidity |= FLAG_LS_PLANE;

	return true;
}

bool Neighbourhood::computeQuadric()
{
	//invalidate previous quadric (if any)
	m_structuresValidity &= (~FLAG_QUADRIC);

	assert(m_associatedCloud);
	if (!m_associatedCloud)
		return false;

	unsigned count = m_associatedCloud->size();

	static_assert(CC_LOCAL_MODEL_MIN_SIZE[QUADRIC] >= 5, "Invalid CC_LOCAL_MODEL_MIN_SIZE size");
	if (count < CC_LOCAL_MODEL_MIN_SIZE[QUADRIC])
	{
		return false;
	}

	const PointCoordinateType* lsPlane = getLSPlane();
	if (!lsPlane)
	{
		return false;
	}

	//we get centroid (should already be up-to-date - see computeCovarianceMatrix)
	const CCVector3* G = getLocalGravityCenter();
	assert(G);

	//get the best projection axis
	Tuple3ub idx(0/*x*/, 1/*y*/, 2/*z*/); //default configuration: z is the "normal" direction, we use (x,y) as the base plane
	const PointCoordinateType nxx = lsPlane[0] * lsPlane[0];
	const PointCoordinateType nyy = lsPlane[1] * lsPlane[1];
	const PointCoordinateType nzz = lsPlane[2] * lsPlane[2];
	if (nxx > nyy)
	{
		if (nxx > nzz)
		{
			//as x is the "normal" direction, we use (y,z) as the base plane
			idx.x = 1/*y*/; idx.y = 2/*z*/; idx.z = 0/*x*/;
		}
	}
	else
	{
		if (nyy > nzz)
		{
			//as y is the "normal" direction, we use (z,x) as the base plane
			idx.x = 2/*z*/; idx.y = 0/*x*/; idx.z = 1/*y*/;
		}
	}

	//compute the A matrix and b vector
	std::vector<PointCoordinateType> A;
	std::vector<PointCoordinateType> b;
	try
	{
		A.resize(6 * count, 0);
		b.resize(count, 0);
	}
	catch (const std::bad_alloc&)
	{
		//not enough memory
		return false;
	}

	PointCoordinateType lmax2 = 0; //max (squared) dimension

	//for all points
	{
		PointCoordinateType* _A = A.data();
		PointCoordinateType* _b = b.data();
		for (unsigned i = 0; i < count; ++i)
		{
			CCVector3 Plocal = *m_associatedCloud->getLocalPoint(i) - *G;

			PointCoordinateType lX = Plocal.u[idx.x];
			PointCoordinateType lY = Plocal.u[idx.y];
			PointCoordinateType lZ = Plocal.u[idx.z];

			*_A++ = 1.0f;
			*_A++ = lX;
			*_A++ = lY;
			*_A = lX * lX;
			//by the way, we track the max 'X' squared dimension
			if (*_A > lmax2)
				lmax2 = *_A;
			++_A;
			*_A++ = lX * lY;
			*_A = lY * lY;
			//by the way, we track the max 'Y' squared dimension
			if (*_A > lmax2)
				lmax2 = *_A;
			++_A;

			*_b++ = lZ;
			lZ *= lZ;
			//and don't forget to track the max 'Z' squared dimension as well
			if (lZ > lmax2)
				lmax2 = lZ;
		}
	}

	//conjugate gradient initialization
	//we solve tA.A.X=tA.b
	ConjugateGradient<6, double> cg;
	const SquareMatrixd& tAA = cg.A();
	double* tAb = cg.b();

	//compute tA.A and tA.b
	{
		for (unsigned i = 0; i < 6; ++i)
		{
			//tA.A part
			for (unsigned j = i; j < 6; ++j)
			{
				double tmp = 0;
				PointCoordinateType* _Ai = &(A[i]);
				PointCoordinateType* _Aj = &(A[j]);
				for (unsigned k = 0; k < count; ++k, _Ai += 6, _Aj += 6)
				{
					//tmp += A[(6*k)+i] * A[(6*k)+j];
					tmp += static_cast<double>(*_Ai) * (*_Aj);
				}
				tAA.m_values[j][i] = tAA.m_values[i][j] = tmp;
			}

			//tA.b part
			{
				double tmp = 0;
				PointCoordinateType* _Ai = &(A[i]);
				for (unsigned k = 0; k < count; ++k, _Ai += 6)
				{
					//tmp += A[(6*k)+i]*b[k];
					tmp += static_cast<double>(*_Ai) * b[k];
				}
				tAb[i] = tmp;
			}
		}
	}

	A.clear();
	b.clear();

	//first guess for X: plane equation (a0.x+a1.y+a2.z=a3 --> z = a3/a2 - a0/a2.x - a1/a2.y)
	double X0[6] = {	static_cast<double>(/*lsPlane[3]/lsPlane[idx.z]*/0), //DGM: warning, points have already been recentred around the gravity center! So forget about a3
						static_cast<double>(-lsPlane[idx.x] / lsPlane[idx.z]),
						static_cast<double>(-lsPlane[idx.y] / lsPlane[idx.z]),
						0,
						0,
						0 };

	//special case: a0 = a1 = a2 = 0! //happens for perfectly flat surfaces!
	if (X0[1] == 0 && X0[2] == 0)
	{
		X0[0] = 1.0;
	}

	//init. conjugate gradient
	cg.initConjugateGradient(X0);

	//conjugate gradient iterations
	{
		const double ConvergenceThreshold = lmax2 * 1.0e-8;  //max. error for convergence = 1e-8 of largest cloud dimension (empirical!)
		for (unsigned i = 0; i < 1500; ++i)
		{
			double lastError = cg.iterConjugateGradient(X0);
			if (lastError < ConvergenceThreshold) //converged
				break;
		}
	}

	//output
	{
		for (unsigned i = 0; i < 6; ++i)
		{
			m_localQuadricEquation[i] = static_cast<PointCoordinateType>(X0[i]);
			m_globalQuadricEquation[i] = X0[i];
		}

		// shift the equation from local to global coordinate system
		CCVector3d shift = m_associatedCloud->getLocalToGlobalTranslation();
		m_globalQuadricEquation[0] = shift[idx.z]
									+ X0[0]
									- X0[1] * shift[idx.x]
									- X0[2] * shift[idx.y]
									+ X0[3] * (shift[idx.x] * shift[idx.x])
									+ X0[4] * (shift[idx.x] * shift[idx.y])
									+ X0[5] * (shift[idx.y] * shift[idx.y]);
		m_globalQuadricEquation[1] -= 2 * X0[3] * shift[idx.x] + X0[4] * shift[idx.y];
		m_globalQuadricEquation[2] -= 2 * X0[5] * shift[idx.y] + X0[4] * shift[idx.x];

		m_structuresValidity |= FLAG_QUADRIC;
	}

	return true;
}

bool Neighbourhood::compute3DQuadric(double quadricEquation[10])
{
	if (!m_associatedCloud || !quadricEquation)
	{
		//invalid (input) parameters
		assert(false);
		return false;
	}

	//computes a 3D quadric of the form ax2 +by2 +cz2 + 2exy + 2fyz + 2gzx + 2lx + 2my + 2nz + d = 0
	//"THREE-DIMENSIONAL SURFACE CURVATURE ESTIMATION USING QUADRIC SURFACE PATCHES", I. Douros & B. Buxton, University College London

	//we get centroid
	const CCVector3* G = getLocalGravityCenter();
	assert(G);

	//we look for the eigen vector associated to the minimum eigen value of a matrix A
	//where A=transpose(D)*D, and D=[xi^2 yi^2 zi^2 xiyi yizi xizi xi yi zi 1] (i=1..N)

	unsigned count = m_associatedCloud->size();

	//we compute M = [x2 y2 z2 xy yz xz x y z 1] for all points
	std::vector<PointCoordinateType> M;
	{
		try
		{
			M.resize(count*10);
		}
		catch (const std::bad_alloc&)
		{
			return false;
		}

		PointCoordinateType* _M = M.data();
		for (unsigned i = 0; i < count; ++i)
		{
			const CCVector3 Plocal = *m_associatedCloud->getLocalPoint(i) - *G;

			//we fill the ith line
			(*_M++) = Plocal.x * Plocal.x;
			(*_M++) = Plocal.y * Plocal.y;
			(*_M++) = Plocal.z * Plocal.z;
			(*_M++) = Plocal.x * Plocal.y;
			(*_M++) = Plocal.y * Plocal.z;
			(*_M++) = Plocal.x * Plocal.z;
			(*_M++) = Plocal.x;
			(*_M++) = Plocal.y;
			(*_M++) = Plocal.z;
			(*_M++) = 1;
		}
	}

	//D = tM.M
	SquareMatrixd D(10);
	for (unsigned l = 0; l < 10; ++l)
	{
		for (unsigned c = 0; c < 10; ++c)
		{
			double sum = 0;
			const PointCoordinateType* _M = M.data();
			for (unsigned i = 0; i < count; ++i, _M += 10)
				sum += static_cast<double>(_M[l] * _M[c]);

			D.m_values[l][c] = sum;
		}
	}

	//we don't need M anymore
	M.resize(0);

	//now we compute eigen values and vectors of D
	SquareMatrixd eigVectors;
	std::vector<double> eigValues;
	if (!Jacobi<double>::ComputeEigenValuesAndVectors(D, eigVectors, eigValues, true))
	{
		//failure
		return false;
	}

	//we get the eigen vector corresponding to the minimum eigen value
	double minEigValue = 0;
	Jacobi<double>::GetMinEigenValueAndVector(eigVectors, eigValues, minEigValue, quadricEquation);

	return true;
}

GenericIndexedMesh* Neighbourhood::triangulateOnPlane( bool duplicateVertices,
													   PointCoordinateType maxEdgeLength,
													   std::string& outputErrorStr )
{
	if (m_associatedCloud->size() < CC_LOCAL_MODEL_MIN_SIZE[TRI])
	{
		//can't compute LSF plane with less than 3 points!
		outputErrorStr = "Not enough points";
		return nullptr;
	}

	//safety check: Triangle lib will crash if the points are all the same!
	if (LessThanEpsilon(computeLargestRadius()))
	{
		return nullptr;
	}

	//project the points on this plane
	GenericIndexedMesh* mesh = nullptr;
	std::vector<CCVector2> points2D;

	if (projectLocalPointsOn2DPlane<CCVector2>(points2D))
	{
		Delaunay2dMesh* dm = new Delaunay2dMesh();

		//triangulate the projected points
		if (!dm->buildMesh(points2D, Delaunay2dMesh::USE_ALL_POINTS, outputErrorStr))
		{
			delete dm;
			return nullptr;
		}

		//change the default mesh's reference
		if (duplicateVertices)
		{
			PointCloud* newVertices = new PointCloud;
			const unsigned count = m_associatedCloud->size();
			if (!newVertices->reserve(count))
			{
				outputErrorStr = "Not enough memory";
				delete dm;
				delete newVertices;
				return nullptr;
			}

			// don't forget to transfer the 'local to global' translation
			newVertices->setLocalToGlobalTranslation(m_associatedCloud->getLocalToGlobalTranslation());

			for (unsigned i = 0; i < count; ++i)
			{
				newVertices->addLocalPoint(*m_associatedCloud->getLocalPoint(i));
			}
			dm->linkMeshWith(newVertices, true);
		}
		else
		{
			dm->linkMeshWith(m_associatedCloud, false);
		}

		//remove triangles with too long edges
		if (maxEdgeLength > 0)
		{
			dm->removeTrianglesWithEdgesLongerThan(maxEdgeLength);
			if (dm->size() == 0)
			{
				//no more triangles?
				outputErrorStr = "Not triangle left after pruning";
				delete dm;
				dm = nullptr;
			}
		}
		mesh = static_cast<GenericIndexedMesh*>(dm);
	}

	return mesh;
}

GenericIndexedMesh* Neighbourhood::triangulateFromQuadric(unsigned nStepX, unsigned nStepY)
{
	if (!m_associatedCloud)
	{
		assert(false);
		return nullptr;
	}

	if (nStepX < 2 || nStepY < 2)
	{
		return nullptr;
	}

	//qaudric fit
	const PointCoordinateType* Q = getLocalQuadric(); //Q: Z = a + b.X + c.Y + d.X^2 + e.X.Y + f.Y^2
	if (!Q)
	{
		return nullptr;
	}

	const PointCoordinateType& a = Q[0];
	const PointCoordinateType& b = Q[1];
	const PointCoordinateType& c = Q[2];
	const PointCoordinateType& d = Q[3];
	const PointCoordinateType& e = Q[4];
	const PointCoordinateType& f = Q[5];

	const unsigned char X = m_quadricEquationDirections.x;
	const unsigned char Y = m_quadricEquationDirections.y;
	const unsigned char Z = m_quadricEquationDirections.z;

	//gravity center (should be ok if the quadric is ok)
	const CCVector3* G = getLocalGravityCenter();
	assert(G);

	//bounding box
	CCVector3 bbMin;
	CCVector3 bbMax;
	m_associatedCloud->getLocalBoundingBox(bbMin, bbMax);
	CCVector3 bboxDiag = bbMax - bbMin;

	//Sample points on Quadric and triangulate them!
	const PointCoordinateType spanX = bboxDiag.u[X];
	const PointCoordinateType spanY = bboxDiag.u[Y];
	const PointCoordinateType stepX = spanX / (nStepX - 1);
	const PointCoordinateType stepY = spanY / (nStepY - 1);

	PointCloud* vertices = new PointCloud;
	if (!vertices->reserve(nStepX*nStepY))
	{
		delete vertices;
		return nullptr;
	}

	// don't forget to transfer the 'local to global' translation
	vertices->setLocalToGlobalTranslation(m_associatedCloud->getLocalToGlobalTranslation());

	SimpleMesh* quadMesh = new SimpleMesh(vertices, true);
	if (!quadMesh->reserve((nStepX - 1)*(nStepY - 1) * 2))
	{
		delete quadMesh;
		return nullptr;
	}

	for (unsigned x = 0; x < nStepX; ++x)
	{
		CCVector3 Plocal;
		Plocal.x = bbMin[X] + stepX * x - G->u[X];
		for (unsigned y = 0; y < nStepY; ++y)
		{
			Plocal.y = bbMin[Y] + stepY * y - G->u[Y];
			Plocal.z = a + (b * Plocal.x) + (c * Plocal.y) + (d * Plocal.x*Plocal.x) + e * (Plocal.x*Plocal.y) + f * (Plocal.y*Plocal.y);

			CCVector3 Pc;
			Pc.u[X] = Plocal.x;
			Pc.u[Y] = Plocal.y;
			Pc.u[Z] = Plocal.z;
			Pc += *G;

			vertices->addLocalPoint(Pc);

			if (x > 0 && y > 0)
			{
				const unsigned iA = (x - 1) * nStepY + y - 1;
				const unsigned iB = iA + 1;
				const unsigned iC = iA + nStepY;
				const unsigned iD = iB + nStepY;

				quadMesh->addTriangle(iA, iC, iB);
				quadMesh->addTriangle(iB, iC, iD);
			}
		}
	}

	return quadMesh;
}

ScalarType Neighbourhood::computeMomentOrder1(const CCVector3& Plocal)
{
	if (!m_associatedCloud || m_associatedCloud->size() < 3)
	{
		//not enough points
		return NAN_VALUE;
	}

	SquareMatrixd eigVectors;
	std::vector<double> eigValues;
	if (!Jacobi<double>::ComputeEigenValuesAndVectors(computeCovarianceMatrix(), eigVectors, eigValues, true))
	{
		//failed to compute the eigen values
		return NAN_VALUE;
	}

	Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues); //sort the eigenvectors in decreasing order of their associated eigenvalues

	double m1 = 0.0;
	double m2 = 0.0;
	CCVector3d e2;
	Jacobi<double>::GetEigenVector(eigVectors, 1, e2.u);

	for (unsigned i = 0; i < m_associatedCloud->size(); ++i)
	{
		double dotProd = (*m_associatedCloud->getLocalPoint(i) - Plocal).toDouble().dot(e2);
		m1 += dotProd;
		m2 += dotProd * dotProd;
	}

	//see "Contour detection in unstructured 3D point clouds", Hackel et al 2016
	return (m2 < std::numeric_limits<double>::epsilon() ? NAN_VALUE : static_cast<ScalarType>((m1 * m1) / m2));
}

double Neighbourhood::computeFeature(GeomFeature feature)
{
	if (!m_associatedCloud || m_associatedCloud->size() < 3)
	{
		//not enough points
		return std::numeric_limits<double>::quiet_NaN();
	}

	SquareMatrixd eigVectors;
	std::vector<double> eigValues;
	if (!Jacobi<double>::ComputeEigenValuesAndVectors(computeCovarianceMatrix(), eigVectors, eigValues, true))
	{
		//failed to compute the eigen values
		return std::numeric_limits<double>::quiet_NaN();
	}

	Jacobi<double>::SortEigenValuesAndVectors(eigVectors, eigValues); //sort the eigenvectors in decreasing order of their associated eigenvalues

	//shortcuts
	const double& l1 = eigValues[0];
	const double& l2 = eigValues[1];
	const double& l3 = eigValues[2];

	double value = std::numeric_limits<double>::quiet_NaN();

	switch (feature)
	{
	case EigenValuesSum:
		value = l1 + l2 + l3;
		break;
	case Omnivariance:
		value = pow(l1 * l2 * l3, 1.0 / 3.0);
		break;
	case EigenEntropy:
		value = -(l1 * log(l1) + l2 * log(l2) + l3 * log(l3));
		break;
	case Anisotropy:
		if (std::abs(l1) > std::numeric_limits<double>::epsilon())
			value = (l1 - l3) / l1;
		break;
	case Planarity:
		if (std::abs(l1) > std::numeric_limits<double>::epsilon())
			value = (l2 - l3) / l1;
		break;
	case Linearity:
		if (std::abs(l1) > std::numeric_limits<double>::epsilon())
			value = (l1 - l2) / l1;
		break;
	case PCA1:
	{
		double sum = l1 + l2 + l3;
		if (std::abs(sum) > std::numeric_limits<double>::epsilon())
			value = l1 / sum;
	}
	break;
	case PCA2:
	{
		double sum = l1 + l2 + l3;
		if (std::abs(sum) > std::numeric_limits<double>::epsilon())
			value = l2 / sum;
	}
	break;
	case SurfaceVariation:
	{
		double sum = l1 + l2 + l3;
		if (std::abs(sum) > std::numeric_limits<double>::epsilon())
			value = l3 / sum;
	}
	break;
	case Sphericity:
		if (std::abs(l1) > std::numeric_limits<double>::epsilon())
			value = l3 / l1;
		break;
	case Verticality:
	{
		CCVector3d Z(0, 0, 1);
		CCVector3d e3(Z);
		Jacobi<double>::GetEigenVector(eigVectors, 2, e3.u);

		value = 1.0 - std::abs(Z.dot(e3));
	}
	break;
	case EigenValue1:
		value = l1;
		break;
	case EigenValue2:
		value = l2;
		break;
	case EigenValue3:
		value = l3;
		break;
	default:
		assert(false);
		break;
	}

	return value;
}

ScalarType Neighbourhood::computeRoughness(const CCVector3& Plocal, const CCVector3* roughnessUpDir/*=nullptr*/)
{
	const PointCoordinateType* lsPlane = getLSPlane();
	if (lsPlane)
	{
		ScalarType distToPlane = DistanceComputationTools::computePoint2PlaneDistance(Plocal, lsPlane);
		if (roughnessUpDir)
		{
			if (CCVector3::vdot(lsPlane, roughnessUpDir->u) < 0)
			{
				distToPlane = -distToPlane;
			}
		}
		else
		{
			distToPlane = std::abs(distToPlane);
		}
		return distToPlane;
	}
	else
	{
		return NAN_VALUE;
	}
}

ScalarType Neighbourhood::computeCurvature(const CCVector3& Plocal, CurvatureType cType)
{
	switch (cType)
	{
	case GAUSSIAN_CURV:
	case MEAN_CURV:
	{
		//we get 2D1/2 quadric parameters
		const PointCoordinateType* H = getLocalQuadric();
		if (!H)
			return NAN_VALUE;

		//compute centroid
		const CCVector3* G = getLocalGravityCenter();

		//we compute curvature at the input neighbour position + we recenter it by the way
		const CCVector3 Q(Plocal - *G);

		const unsigned char X = m_quadricEquationDirections.x;
		const unsigned char Y = m_quadricEquationDirections.y;

		//z = a+b.x+c.y+d.x^2+e.x.y+f.y^2
		//const PointCoordinateType& a = H[0];
		const PointCoordinateType& b = H[1];
		const PointCoordinateType& c = H[2];
		const PointCoordinateType& d = H[3];
		const PointCoordinateType& e = H[4];
		const PointCoordinateType& f = H[5];

		//See "CURVATURE OF CURVES AND SURFACES – A PARABOLIC APPROACH" by ZVI HAR’EL
		const PointCoordinateType  fx = b + (d * 2) * Q.u[X] + (e)* Q.u[Y];	// b+2d*X+eY
		const PointCoordinateType  fy = c + (e)* Q.u[X] + (f * 2) * Q.u[Y];	// c+2f*Y+eX
		const PointCoordinateType  fxx = d * 2;									// 2d
		const PointCoordinateType  fyy = f * 2;									// 2f
		const PointCoordinateType& fxy = e;									// e

		const PointCoordinateType fx2 = fx * fx;
		const PointCoordinateType fy2 = fy * fy;
		const PointCoordinateType q = (1 + fx2 + fy2);

		switch (cType)
		{
		case GAUSSIAN_CURV:
		{
			//to sign the curvature, we need a normal!
			const PointCoordinateType K = std::abs(fxx*fyy - fxy * fxy) / (q*q);
			return static_cast<ScalarType>(K);
		}

		case MEAN_CURV:
		{
			//to sign the curvature, we need a normal!
			const PointCoordinateType H2 = std::abs(((1 + fx2)*fyy - 2 * fx*fy*fxy + (1 + fy2)*fxx)) / (2 * sqrt(q)*q);
			return static_cast<ScalarType>(H2);
		}

		default:
			assert(false);
			break;
		}
	}
	break;

	case NORMAL_CHANGE_RATE:
	{
		assert(m_associatedCloud);
		unsigned pointCount = (m_associatedCloud ? m_associatedCloud->size() : 0);

		//we need at least 4 points
		if (pointCount < 4)
		{
			//not enough points!
			return pointCount == 3 ? 0 : NAN_VALUE;
		}

		//we determine plane normal by computing the smallest eigen value of M = 1/n * S[(p-µ)*(p-µ)']
		SquareMatrixd covMat = computeCovarianceMatrix();
		CCVector3d e(0, 0, 0);

		SquareMatrixd eigVectors;
		std::vector<double> eigValues;
		if (!Jacobi<double>::ComputeEigenValuesAndVectors(covMat, eigVectors, eigValues, true))
		{
			//failure
			return NAN_VALUE;
		}

		//compute curvature as the rate of change of the surface
		e.x = eigValues[0];
		e.y = eigValues[1];
		e.z = eigValues[2];

		const double sum = e.x + e.y + e.z; //we work with absolute values
		if (LessThanEpsilon(sum))
		{
			return NAN_VALUE;
		}

		const double eMin = std::min(std::min(e.x, e.y), e.z);
		return static_cast<ScalarType>(eMin / sum);
	}
	break;

	default:
		assert(false);
	}

	return NAN_VALUE;
}
