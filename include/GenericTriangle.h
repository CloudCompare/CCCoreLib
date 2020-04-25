// SPDX-License-Identifier: LGPL-2.0-or-later
// Copyright © EDF R&D / TELECOM ParisTech (ENST-TSI)

#pragma once

//Local
#include "CCGeom.h"

namespace CCCoreLib
{
	//! A generic triangle interface
	/** Returns (temporary) references to each vertex.
	**/
	class CC_CORE_LIB_API GenericTriangle
	{
	public:

		//! Default destructor
		virtual ~GenericTriangle() = default;

		//! Returns the first vertex (A)
		virtual const CCVector3* _getA() const = 0;

		//! Returns the second vertex (B)
		virtual const CCVector3* _getB() const = 0;

		//! Returns the third vertex (C)
		virtual const CCVector3* _getC() const = 0;
	};
}
