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
	template <typename Type> class CC_CORE_LIB_API GenericTriangleTpl
	{
	public:

		//! Default destructor
		virtual ~GenericTriangleTpl() = default;

		//! Returns the first vertex (A)
		virtual const Vector3Tpl<Type>* _getA() const = 0;

		//! Returns the second vertex (B)
		virtual const Vector3Tpl<Type>* _getB() const = 0;

		//! Returns the third vertex (C)
		virtual const Vector3Tpl<Type>* _getC() const = 0;
	};

	//! Default local triangle
	using GenericLocalTriangle = GenericTriangleTpl<PointCoordinateType>;
	//! Default global triangle
	using GenericGlobalTriangle = GenericTriangleTpl<double>;

}
