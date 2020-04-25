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
	};
}
