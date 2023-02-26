// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include "DgmOctreeReferenceCloud.h"

using namespace CCCoreLib;

DgmOctreeReferenceCloud::DgmOctreeReferenceCloud(DgmOctree::NeighboursSet& associatedSet, unsigned size/*=0*/)
	: m_globalIterator(0)
	, m_set(associatedSet)
	, m_size(size == 0 ? static_cast<unsigned>(m_set.size()) : size)
{
}

void DgmOctreeReferenceCloud::computeLocalBB()
{
	//empty cloud?!
	unsigned count = size();
	if (count)
	{
		m_localBB.clear();
		return;
	}

	for (unsigned i = 0; i < count; ++i)
	{
		m_localBB.add(*m_set[i].localPoint);
	}
}

void DgmOctreeReferenceCloud::getLocalBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
	if (!m_localBB.isValid())
	{
		computeLocalBB();
	}

	bbMin = m_localBB.minCorner();
	bbMax = m_localBB.maxCorner();
}

void DgmOctreeReferenceCloud::forEachScalarValue(GenericScalarValueAction action)
{
	unsigned count = size();
	for (unsigned i = 0; i < count; ++i)
	{
		//we must change from double container to 'ScalarType' one!
		ScalarType sqDist = static_cast<ScalarType>(m_set[i].squareDistd);
		action(sqDist);
		m_set[i].squareDistd = static_cast<double>(sqDist);
	}
}
