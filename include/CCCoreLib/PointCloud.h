// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#ifndef CC_LIB_POINT_CLOUD_HEADER
#define CC_LIB_POINT_CLOUD_HEADER

//Local
#include "GenericIndexedCloudPersist.h"
#include "PointCloudTpl.h"

namespace CCLib
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

#endif //CC_LIB_POINT_CLOUD_HEADER
