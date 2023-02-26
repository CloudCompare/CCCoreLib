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

	public: // normals management

		// Inherited from PointCloudTpl
		bool resize(unsigned newNumberOfPoints) override;

		//! Reserves memory to store the normals
		bool reserveNormals(unsigned newCount);

		//! Adds a normal
		/** \param N a 3D normal
		**/
		void addNormal(const CCVector3& N);

		//! Returns the set of normals
		std::vector<CCVector3>& normals() { return m_normals; }

		//! Returns the set of normals (const version)
		const std::vector<CCVector3>& normals() const { return m_normals; }

		// Inherited from CCCoreLib::GenericIndexedCloud
		bool normalsAvailable() const override { return !m_normals.empty() && m_normals.size() >= size(); }
		const CCVector3* getNormal(unsigned pointIndex) const override { return &m_normals[pointIndex]; }

	protected:

		//! Point normals
		std::vector<CCVector3> m_normals;
	};
}
