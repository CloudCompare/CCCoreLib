// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "GenericIndexedCloudPersist.h"
#include "PointCloudTpl.h"

namespace CCCoreLib
{
	//! A storage-efficient point cloud structure that can also handle an unlimited number of scalar fields
	class CC_CORE_LIB_API PointCloud : public PointCloudTpl<GenericIndexedCloudPersist>
	{
	public:
		//! Default constructor
		PointCloud() = default;

		//! Default destructor
		~PointCloud() override = default;

		//! Reserves memory to store the normals
		bool reserveNormals(unsigned newCount)
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

		//inherited from PointCloudTpl
		bool resize(unsigned newNumberOfPoints) override
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

		//! Adds a normal
		/** \param N a 3D normal
		**/
		inline void addNormal(const CCVector3 &N)
		{
			assert(m_normals.size() < m_normals.capacity());
			m_normals.push_back(N);
		}

		//! Returns the set of normals
		std::vector<CCVector3>& normals() { return m_normals; }

		//! Returns the set of normals (const version)
		const std::vector<CCVector3>& normals() const { return m_normals; }

		//inherited from CCCoreLib::GenericIndexedCloud
		bool normalsAvailable() const override { return !m_normals.empty() && m_normals.size() >= size(); }
		const CCVector3* getNormal(unsigned pointIndex) const override { return &m_normals[pointIndex]; }

	protected:

		//! Point normals (if any)
		std::vector<CCVector3> m_normals;
	};
}
