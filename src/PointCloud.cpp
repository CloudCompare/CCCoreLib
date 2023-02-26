// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#include "../include/PointCloud.h"

using namespace CCCoreLib;

bool PointCloud::resize(unsigned newNumberOfPoints)
{
	if (!PointCloudTpl<GenericIndexedCloudPersist>::resize(newNumberOfPoints))
	{
		return false;
	}

	// resize the normals as well
	if (m_normals.capacity() != 0)
	{
		try
		{
			m_normals.resize(newNumberOfPoints);
		}
		catch (const std::bad_alloc&)
		{
			return false;
		}
	}

	return true;
}

bool PointCloud::reserveNormals(unsigned newCount)
{
	if (m_normals.capacity() < newCount)
	{
		try
		{
			m_normals.reserve(newCount);
		}
		catch (const std::bad_alloc&)
		{
			return false;
		}
	}

	return true;
}

void PointCloud::addNormal(const CCVector3& N)
{
	assert(m_normals.size() < m_normals.capacity());
	m_normals.push_back(N);
}
