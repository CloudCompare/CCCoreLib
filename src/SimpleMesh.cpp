// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include "SimpleMesh.h"

//local
#include "GenericIndexedCloud.h"

//System
#include <cassert>

using namespace CCCoreLib;

SimpleMesh::SimpleMesh(GenericIndexedCloud* _theVertices, bool linkVerticesWithMesh)
	: GenericIndexedMesh()
	, globalIterator(0)
	, theVertices(_theVertices)
	, verticesLinked(linkVerticesWithMesh)
{
}

SimpleMesh::~SimpleMesh()
{
	if (theVertices && verticesLinked)
	{
		delete theVertices;
		theVertices = nullptr;
	}
}

void SimpleMesh::forEach(genericTriangleAction action)
{
	SimpleTriangle tri;

	for (VerticesIndexes& ti : triIndexes)
	{
		theVertices->getPoint(ti.i1, tri.A);
		theVertices->getPoint(ti.i2, tri.B);
		theVertices->getPoint(ti.i3, tri.C);
		action(tri);
	}
}

void SimpleMesh::placeIteratorAtBeginning()
{
	globalIterator = 0;
}

GenericTriangle* SimpleMesh::_getNextTriangle()
{
	return _getTriangle(globalIterator++);
}

GenericTriangle* SimpleMesh::_getTriangle(unsigned triangleIndex)
{
	assert(triangleIndex < triIndexes.size());

	const VerticesIndexes& ti = triIndexes[triangleIndex];
	theVertices->getPoint(ti.i1, dummyTriangle.A);
	theVertices->getPoint(ti.i2, dummyTriangle.B);
	theVertices->getPoint(ti.i3, dummyTriangle.C);

	return &dummyTriangle; //temporary!
}

void SimpleMesh::getTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const
{
	assert(triangleIndex < triIndexes.size());

	const VerticesIndexes& ti = triIndexes[triangleIndex];
	theVertices->getPoint(ti.i1, A);
	theVertices->getPoint(ti.i2, B);
	theVertices->getPoint(ti.i3, C);
}

void SimpleMesh::getBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
	////TODO: how can we know if the vertices cloud changes?!
	//if (!m_bbox.isValid())
	//{
	//	m_bbox.clear();
	//	for (const VerticesIndexes& ti : triIndexes)
	//	{
	//		m_bbox.add(*theVertices->getPoint(ti.i1));
	//		m_bbox.add(*theVertices->getPoint(ti.i2));
	//		m_bbox.add(*theVertices->getPoint(ti.i3));
	//	}
	//}

	//bbMin = m_bbox.minCorner();
	//bbMax = m_bbox.maxCorner();

	return theVertices->getBoundingBox(bbMin, bbMax);
}

void SimpleMesh::addTriangle(unsigned i1, unsigned i2, unsigned i3)
{
	triIndexes.push_back(VerticesIndexes(i1, i2, i3));

	m_bbox.setValidity(false);
}

bool SimpleMesh::reserve(unsigned n)
{
	try
	{
		triIndexes.reserve(n);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}
	return true;
}

bool SimpleMesh::resize(unsigned n)
{
	try
	{
		triIndexes.resize(n);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}
	return true;
}

bool SimpleMesh::normalsAvailable() const
{
	return theVertices && theVertices->normalsAvailable();
}

bool SimpleMesh::interpolateNormals(unsigned triIndex, const CCVector3& P, CCVector3& N)
{
	if (static_cast<size_t>(triIndex) >= triIndexes.size())
	{
		// index out of range
		assert(false);
		return false;
	}
	
	const VerticesIndexes& tri = triIndexes[triIndex];

	// intepolation weights
	CCVector3d weights;
	{
		CCVector3 A, B, C;
		theVertices->getPoint(tri.i1, A);
		theVertices->getPoint(tri.i2, B);
		theVertices->getPoint(tri.i3, C);

		// barcyentric intepolation weights
		weights.x = sqrt(((P - B).cross(C - B)).norm2d())/*/2*/;
		weights.y = sqrt(((P - C).cross(A - C)).norm2d())/*/2*/;
		weights.z = sqrt(((P - A).cross(B - A)).norm2d())/*/2*/;

		// normalize weights
		double sum = weights.x + weights.y + weights.z;
		weights /= sum;
	}

	// interpolated normal
	CCVector3d Nd(0, 0, 0);
	{
		const CCVector3* N1 = theVertices->getNormal(tri.i1);
		Nd += N1->toDouble() * weights.u[0];

		const CCVector3* N2 = theVertices->getNormal(tri.i2);
		Nd += N2->toDouble() * weights.u[1];

		const CCVector3* N3 = theVertices->getNormal(tri.i3);
		Nd += N3->toDouble() * weights.u[2];

		Nd.normalize();
	}

	N = Nd.toPC();

	return true;
}

VerticesIndexes* SimpleMesh::getTriangleVertIndexes(unsigned triangleIndex)
{
	return &(triIndexes[triangleIndex]);
}

VerticesIndexes* SimpleMesh::getNextTriangleVertIndexes()
{
	return getTriangleVertIndexes(globalIterator++);
}
