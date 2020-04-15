// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#ifndef GENERIC_OCTREE_HEADER
#define GENERIC_OCTREE_HEADER

//Local
#include "CCGeom.h"

namespace CCLib
{

	//! A generic octree interface for data communication between library and client applications
	class CC_CORE_LIB_API GenericOctree
	{
	public:

		//! Default destructor
		virtual ~GenericOctree() = default;
	};

}

#endif //GENERIC_OCTREE_HEADER
