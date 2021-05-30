// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "GenericIndexedCloud.h"

namespace CCCoreLib
{
	//! A generic 3D point cloud with index-based and presistent access to points
	/** Implements the GenericIndexedCloud interface.
	**/
	class CC_CORE_LIB_API GenericIndexedCloudPersist : virtual public GenericIndexedCloud
	{
	public:
		//! Default constructor
		GenericIndexedCloudPersist() = default;

		//! Mock constructor for compatibility with the PointCloudTpl interface
		/** \warning Parameters are simply ignored
			\param name ignored
			\param ID ignored
		**/
		GenericIndexedCloudPersist(const char* name, unsigned ID) { (void)name; (void)ID; /* input parameters are ignored */ }


		//! Default destructor
		~GenericIndexedCloudPersist() override = default;

		//! Returns the ith point as a persistent pointer
		/**	Virtual method to request a point with a specific index.
			WARNING: the returned object MUST be persistent in order
			to be compatible with parallel strategies!
			\param index of the requested point (between 0 and the cloud size minus 1)
			\return the requested point (or 0 if index is invalid)
		**/
		virtual const CCVector3* getPointPersistentPtr(unsigned index) const = 0;
	};
}
