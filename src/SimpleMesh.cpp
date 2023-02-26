// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include "SimpleMesh.h"

//local
#include "GenericIndexedCloud.h"

//System
#include <cassert>

using namespace CCCoreLib;

SimpleMesh::SimpleMesh(GenericIndexedCloud* vertices, bool linkVerticesWithMesh)
	: GenericIndexedMesh()
	, m_globalIterator(0)
	, m_vertices(vertices)
	, m_verticesLinked(linkVerticesWithMesh)
{
}

SimpleMesh::~SimpleMesh()
{
	if (m_vertices && m_verticesLinked)
	{
		delete m_vertices;
		m_vertices = nullptr;
	}
}

void SimpleMesh::placeIteratorAtBeginning()
{
	m_globalIterator = 0;
}

GenericLocalTriangle* SimpleMesh::_getLocalTriangle(unsigned triangleIndex)
{
	assert(triangleIndex < m_triIndexes.size());

	const VerticesIndexes& ti = m_triIndexes[triangleIndex];
	m_vertices->getLocalPoint(ti.i1, m_dummyLocalTriangle.A);
	m_vertices->getLocalPoint(ti.i2, m_dummyLocalTriangle.B);
	m_vertices->getLocalPoint(ti.i3, m_dummyLocalTriangle.C);

	return &m_dummyLocalTriangle; //temporary!
}

GenericGlobalTriangle* SimpleMesh::_getGlobalTriangle(unsigned triangleIndex)
{
	assert(triangleIndex < m_triIndexes.size());

	const VerticesIndexes& ti = m_triIndexes[triangleIndex];
	m_vertices->getGlobalPoint(ti.i1, m_dummyGlobalTriangle.A);
	m_vertices->getGlobalPoint(ti.i2, m_dummyGlobalTriangle.B);
	m_vertices->getGlobalPoint(ti.i3, m_dummyGlobalTriangle.C);

	return &m_dummyGlobalTriangle; //temporary!
}

void SimpleMesh::getLocalTriangleVertices(unsigned triangleIndex, CCVector3& A, CCVector3& B, CCVector3& C) const
{
	assert(triangleIndex < m_triIndexes.size());

	const VerticesIndexes& ti = m_triIndexes[triangleIndex];
	m_vertices->getLocalPoint(ti.i1, A);
	m_vertices->getLocalPoint(ti.i2, B);
	m_vertices->getLocalPoint(ti.i3, C);
}

void SimpleMesh::getGlobalTriangleVertices(unsigned triangleIndex, CCVector3d& A, CCVector3d& B, CCVector3d& C) const
{
	assert(triangleIndex < m_triIndexes.size());

	const VerticesIndexes& ti = m_triIndexes[triangleIndex];
	m_vertices->getGlobalPoint(ti.i1, A);
	m_vertices->getGlobalPoint(ti.i2, B);
	m_vertices->getGlobalPoint(ti.i3, C);
}

void SimpleMesh::getLocalBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
	if (m_vertices)
	{
		m_vertices->getLocalBoundingBox(bbMin, bbMax);
	}
	else
	{
		assert(false);
	}
}

void SimpleMesh::getGlobalBoundingBox(CCVector3d& bbMin, CCVector3d& bbMax)
{
	if (m_vertices)
	{
		m_vertices->getGlobalBoundingBox(bbMin, bbMax);
	}
	else
	{
		assert(false);
	}
}

CCVector3d SimpleMesh::getLocalToGlobalTranslation() const
{
	return m_vertices ? m_vertices->getLocalToGlobalTranslation() : CCVector3d(0, 0, 0);
}

CCVector3d SimpleMesh::toGlobal(const CCVector3& localPoint) const
{
	return m_vertices ? m_vertices->toGlobal(localPoint) : CCVector3d(0, 0, 0);
}

CCVector3 SimpleMesh::toLocal(const CCVector3d& globalPoint) const
{
	return m_vertices ? m_vertices->toLocal(globalPoint) : CCVector3(0, 0, 0);
}

void SimpleMesh::addTriangle(unsigned i1, unsigned i2, unsigned i3)
{
	m_triIndexes.push_back(VerticesIndexes(i1, i2, i3));

	m_bbox.setValidity(false);
}

bool SimpleMesh::reserve(unsigned n)
{
	try
	{
		m_triIndexes.reserve(n);
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
		m_triIndexes.resize(n);
	}
	catch (const std::bad_alloc&)
	{
		return false;
	}
	return true;
}

bool SimpleMesh::normalsAvailable() const
{
	return m_vertices && m_vertices->normalsAvailable();
}

bool SimpleMesh::interpolateNormalsLocal(unsigned triIndex, const CCVector3& Plocal, CCVector3& N)
{
	if (!m_vertices)
	{
		assert(false);
		return false;
	}
	if (static_cast<size_t>(triIndex) >= m_triIndexes.size())
	{
		// index out of range
		assert(false);
		return false;
	}
	
	const VerticesIndexes& tri = m_triIndexes[triIndex];

	// intepolation weights
	CCVector3d weights;
	{
		CCVector3 A, B, C;
		m_vertices->getLocalPoint(tri.i1, A);
		m_vertices->getLocalPoint(tri.i2, B);
		m_vertices->getLocalPoint(tri.i3, C);

		// barcyentric intepolation weights
		weights.x = sqrt(((Plocal - B).cross(C - B)).norm2d())/*/2*/;
		weights.y = sqrt(((Plocal - C).cross(A - C)).norm2d())/*/2*/;
		weights.z = sqrt(((Plocal - A).cross(B - A)).norm2d())/*/2*/;

		// normalize weights
		double sum = weights.x + weights.y + weights.z;
		weights /= sum;
	}

	// interpolated normal
	CCVector3d Nd(0, 0, 0);
	{
		const CCVector3* N1 = m_vertices->getNormal(tri.i1);
		Nd += N1->toDouble() * weights.u[0];

		const CCVector3* N2 = m_vertices->getNormal(tri.i2);
		Nd += N2->toDouble() * weights.u[1];

		const CCVector3* N3 = m_vertices->getNormal(tri.i3);
		Nd += N3->toDouble() * weights.u[2];

		Nd.normalize();
	}

	N = Nd.toPC();

	return true;
}

bool SimpleMesh::interpolateNormalsGlobal(unsigned triIndex, const CCVector3d& Pglobal, CCVector3& N)
{
	if (!m_vertices)
	{
		assert(false);
		return false;
	}

	return interpolateNormalsLocal(triIndex, m_vertices->toLocal(Pglobal), N);
}
