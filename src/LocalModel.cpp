// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include "LocalModel.h"

//local
#include "DistanceComputationTools.h"
#include "GenericMesh.h"
#include "GenericTriangle.h"


using namespace CCCoreLib;

//! Least Squares Best Fitting Plane "local modelization"
class LSLocalModel : public LocalModel
{
public:

	//! Constructor
	LSLocalModel(const PointCoordinateType eq[4], const CCVector3 &center, PointCoordinateType squaredRadius)
		: LocalModel(center, squaredRadius)
	{
		memcpy(m_eq, eq, sizeof(PointCoordinateType) * 4);
	}

	//inherited from LocalModel
	LOCAL_MODEL_TYPES getType() const override { return LS; }

	//inherited from LocalModel
	ScalarType computeDistanceFromModelToPoint(const CCVector3* P, CCVector3* nearestPoint = nullptr) const override
	{
		ScalarType dist = DistanceComputationTools::computePoint2PlaneDistance(P, m_eq);

		if (nearestPoint)
		{
			*nearestPoint = *P - static_cast<PointCoordinateType>(dist) * CCVector3(m_eq);
		}

		return std::abs(dist);
	}

protected:

	//! Plane equation
	PointCoordinateType m_eq[4];
};

//! Delaunay 2D1/2 "local modelization"
class DelaunayLocalModel : public LocalModel
{
public:

	//! Constructor
	DelaunayLocalModel(GenericMesh* tri, const CCVector3 &center, PointCoordinateType squaredRadius)
		: LocalModel(center, squaredRadius)
		, m_tri(tri)
	{
		assert(tri);
	}

	//! Destructor
	~DelaunayLocalModel() override {  delete m_tri; }

	//inherited from LocalModel
	LOCAL_MODEL_TYPES getType() const override { return TRI; }

	//inherited from LocalModel
	ScalarType computeDistanceFromModelToPoint(const CCVector3* P, CCVector3* nearestPoint = nullptr) const override
	{
		ScalarType minDist2 = NAN_VALUE;
		if (m_tri)
		{
			m_tri->placeIteratorAtBeginning();
			unsigned numberOfTriangles = m_tri->size();
			CCVector3 triNearestPoint;
			for (unsigned i = 0; i < numberOfTriangles; ++i)
			{
				GenericTriangle* tri = m_tri->_getNextTriangle();
				ScalarType dist2 = DistanceComputationTools::computePoint2TriangleDistance(P, tri, false, nearestPoint ? &triNearestPoint : nullptr);
				if (dist2 < minDist2 || i == 0)
				{
					//keep track of the smallest distance
					minDist2 = dist2;
					if (nearestPoint)
						*nearestPoint = triNearestPoint;
				}
			}
		}

		//there should be at least one triangle!
		assert(minDist2 == minDist2);

		return sqrt(minDist2);
	}

protected:

	//! Associated triangulation
	GenericMesh* m_tri;
};

//! Quadric "local modelization"
/** Former 'Height Function' model.
**/
class QuadricLocalModel : public LocalModel
{
public:

	//! Constructor
	QuadricLocalModel(	const PointCoordinateType eq[6],
						const SquareMatrix& localOrientation,
						const CCVector3& gravityCenter,
						const CCVector3 &center,
						PointCoordinateType squaredRadius )
		: LocalModel(center, squaredRadius)
		, m_localOrientation(localOrientation)
		, m_gravityCenter(gravityCenter)
	{
		m_localOrientationInv = m_localOrientation.inv();
		memcpy(m_eq, eq, sizeof(PointCoordinateType) * 6);
	}

	//inherited from LocalModel
	LOCAL_MODEL_TYPES getType() const override { return QUADRIC; }

	//inherited from LocalModel
	ScalarType computeDistanceFromModelToPoint(const CCVector3* _P, CCVector3* nearestPoint = nullptr) const override
	{
		CCVector3 Plocal = m_localOrientation * (*_P - m_gravityCenter);

		PointCoordinateType z =   m_eq[0]
								+ m_eq[1] * Plocal.x
								+ m_eq[2] * Plocal.y
								+ m_eq[3] * Plocal.x * Plocal.x
								+ m_eq[4] * Plocal.x * Plocal.y
								+ m_eq[5] * Plocal.y * Plocal.y;

		if (nearestPoint)
		{
			*nearestPoint = m_localOrientationInv * CCVector3(Plocal.x, Plocal.y, z);
		}

		return static_cast<ScalarType>(std::abs(Plocal.z - z));
	}

protected:

	//! Quadric equation
	PointCoordinateType m_eq[6];
	//! Quadric local 'orientation' system
	SquareMatrix m_localOrientation;
	//! Quadric local 'orientation' system (inverse)
	SquareMatrix m_localOrientationInv;
	//! Model gravity center
	CCVector3 m_gravityCenter;

};

LocalModel::LocalModel(const CCVector3 &center, PointCoordinateType squaredRadius)
	: m_modelCenter(center)
	, m_squaredRadius(squaredRadius)
{}

LocalModel* LocalModel::New(LOCAL_MODEL_TYPES type,
							Neighbourhood& subset,
							const CCVector3 &center,
							PointCoordinateType squaredRadius)
{
	switch (type)
	{
		case NO_MODEL:
			assert(false);
			break;

		case LS:
		{
			const PointCoordinateType* lsPlane = subset.getLSPlane();
			if (lsPlane)
			{
				return new LSLocalModel(lsPlane, center, squaredRadius);
			}
		}
			break;

		case TRI:
		{
			std::string	errorStr;

			GenericMesh* tri = subset.triangulateOnPlane( Neighbourhood::DUPLICATE_VERTICES,
														  Neighbourhood::IGNORE_MAX_EDGE_LENGTH,
														  errorStr ); //'subset' is potentially associated to a volatile ReferenceCloud, so we must duplicate vertices!
			if (tri)
			{
				return new DelaunayLocalModel(tri, center, squaredRadius);
			}
		}
			break;

		case QUADRIC:
		{
			SquareMatrix toLocalCS;
			const PointCoordinateType* eq = subset.getQuadric(&toLocalCS);
			if (eq)
			{
				return new QuadricLocalModel(	eq,
												toLocalCS,
												*subset.getGravityCenter(), //should be ok as the quadric computation succeeded!
												center,
												squaredRadius );
			}
		}
			break;
	}

	//invalid input type or computation failed!
	return nullptr;
}
