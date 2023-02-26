// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include "ReferenceCloud.h"

using namespace CCCoreLib;

ReferenceCloud::ReferenceCloud(GenericIndexedCloudPersist* associatedCloud)
	: m_globalIterator(0)
	, m_theAssociatedCloud(associatedCloud)
{
}

ReferenceCloud::ReferenceCloud(const ReferenceCloud& refCloud)
	: m_theIndexes(refCloud.m_theIndexes) //we don't catch any exception so that the caller of the constructor can do it!
	, m_globalIterator(0)
	, m_theAssociatedCloud(refCloud.m_theAssociatedCloud)
{
}

void ReferenceCloud::clear(bool releaseMemory/*=false*/)
{
	m_mutex.lock();
	if (releaseMemory)
		m_theIndexes.resize(0);
	else
		m_theIndexes.clear();

	invalidateBoundingBoxInternal(false);
	m_mutex.unlock();
}

void ReferenceCloud::getLocalBoundingBox(CCVector3& bbMin, CCVector3& bbMax)
{
	m_mutex.lock();

	if (!m_bbox.isValid())
	{
		m_bbox.clear();
		for (unsigned index : m_theIndexes)
		{
			m_bbox.add(*m_theAssociatedCloud->getLocalPoint(index));
		}
	}

	bbMin = m_bbox.minCorner();
	bbMax = m_bbox.maxCorner();

	m_mutex.unlock();
}

bool ReferenceCloud::reserve(unsigned n)
{
	m_mutex.lock();
	try
	{
		m_theIndexes.reserve(n);
	}
	catch (const std::bad_alloc&)
	{
		m_mutex.unlock();
		return false;
	}

	m_mutex.unlock();
	return true;
}

bool ReferenceCloud::resize(unsigned n)
{
	m_mutex.lock();
	try
	{
		m_theIndexes.resize(n);
	}
	catch (const std::bad_alloc&)
	{
		m_mutex.unlock();
		return false;
	}

	m_mutex.unlock();
	return true;
}

const CCVector3* ReferenceCloud::getCurrentPointCoordinates() const
{
	assert(m_theAssociatedCloud && m_globalIterator < size());
	assert(m_theIndexes[m_globalIterator] < m_theAssociatedCloud->size());
	return m_theAssociatedCloud->getLocalPointPersistentPtr(m_theIndexes[m_globalIterator]);
}

bool ReferenceCloud::addPointIndex(unsigned globalIndex)
{
	m_mutex.lock();
	try
	{
		m_theIndexes.push_back(globalIndex);
	}
	catch (const std::bad_alloc&)
	{
		m_mutex.unlock();
		return false;
	}
	invalidateBoundingBoxInternal(false);

	m_mutex.unlock();
	return true;
}

bool ReferenceCloud::addPointIndex(unsigned firstIndex, unsigned lastIndex)
{
	if (firstIndex >= lastIndex)
	{
		assert(false);
		return false;
	}

	unsigned range = lastIndex - firstIndex; //lastIndex is excluded

	m_mutex.lock();
	unsigned pos = size();
	if (size() < pos + range)
	{
		try
		{
			m_theIndexes.resize(pos + range);
		}
		catch (const std::bad_alloc&)
		{
			m_mutex.unlock();
			return false;
		}
	}

	for (unsigned i = 0; i < range; ++i, ++firstIndex)
	{
		m_theIndexes[pos++] = firstIndex;
	}

	invalidateBoundingBoxInternal(false);
	m_mutex.unlock();

	return true;
}

void ReferenceCloud::setPointIndex(unsigned localIndex, unsigned globalIndex)
{
	assert(localIndex < size());
	m_theIndexes[localIndex] = globalIndex;
	invalidateBoundingBox();
}

void ReferenceCloud::forEachScalarValue(GenericScalarValueAction action)
{
	assert(m_theAssociatedCloud);

	unsigned count = size();
	for (unsigned i = 0; i < count; ++i)
	{
		unsigned index = m_theIndexes[i];
		ScalarType d = m_theAssociatedCloud->getPointScalarValue(index);
		ScalarType d2 = d;
		action(d2);

		// Since a call to setPointScalarValue can take quite some time
		// (we don't know its implementation), we check first if the value has
		// actually changed before calling it.
		if (d != d2)
		{
			m_theAssociatedCloud->setPointScalarValue(index, d2);
		}
	}
}

void ReferenceCloud::removePointGlobalIndex(unsigned localIndex)
{
	m_mutex.lock();

	if (localIndex < size())
	{
		//swap the value to be removed with the last one
		m_theIndexes[localIndex] = m_theIndexes.back();
		m_theIndexes.pop_back();
	}
	else
	{
		assert(false);
	}

	m_mutex.unlock();
}

void ReferenceCloud::setAssociatedCloud(GenericIndexedCloudPersist* cloud)
{
	m_theAssociatedCloud = cloud;
	invalidateBoundingBox();
}

bool ReferenceCloud::add(const ReferenceCloud& cloud)
{
	if (!cloud.m_theAssociatedCloud || m_theAssociatedCloud != cloud.m_theAssociatedCloud)
	{
		return false;
	}

	std::size_t newCount = cloud.m_theIndexes.size();
	if (newCount == 0)
	{
		return true;
	}

	m_mutex.lock();

	//reserve memory
	std::size_t currentSize = size();
	try
	{
		m_theIndexes.resize(currentSize + newCount);
	}
	catch (const std::bad_alloc&)
	{
		m_mutex.unlock();
		return false;
	}

	//copy new indexes (warning: no duplicate check!)
	for (unsigned i = 0; i < newCount; ++i)
	{
		m_theIndexes[currentSize + i] = cloud.m_theIndexes[i];
	}

	invalidateBoundingBoxInternal(false);

	m_mutex.unlock();
	return true;
}

void ReferenceCloud::invalidateBoundingBoxInternal(bool threadSafe)
{
	if (threadSafe)
	{
		m_mutex.lock();
	}

	m_bbox.setValidity(false);

	if (threadSafe)
	{
		m_mutex.unlock();
	}
}
